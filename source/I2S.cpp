#include "mbed-drivers/I2S.h"
#include "minar/minar.h"
#include "mbed-drivers/mbed_assert.h"
#include "core-util/CriticalSectionLock.h"

#if DEVICE_I2S

namespace mbed {

using namespace util;

#if TRANSACTION_QUEUE_SIZE_I2S
CircularBuffer<I2S::transaction_t, TRANSACTION_QUEUE_SIZE_I2S> I2S::_transaction_buffer;
#endif

I2S::I2S(PinName dpin, PinName clk, PinName wsel, PinName fdpin, PinName mck) :
        _i2s(),
	_irq_tx(this), _irq_rx(this),
        _dbits(16),
        _fbits(16),
	_polarity(0),
	_protocol(PHILIPS),
	_mode(MASTER_TX),
	_busy(false),
        _hz(44100) {
    i2s_init(&_i2s, dpin, clk, wsel, fdpin, mck, _mode);
    i2s_format(&_i2s, _dbits, _fbits, _polarity);
    i2s_audio_frequency(&_i2s, _hz);
    i2s_set_protocol(&_i2s, _protocol);
}

void I2S::format(int dbits, int fbits, int polarity) {
    _dbits = dbits;
    _fbits = fbits;
    _polarity = polarity;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

void I2S::audio_frequency(unsigned int hz) {
    _hz = hz;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

void I2S::set_protocol(i2s_bitorder_t protocol) {
    _protocol = protocol;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

void I2S::set_mode(i2s_mode_t mode) {
	_mode = mode;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

I2S* I2S::_owner = NULL;

// ignore the fact there are multiple physical i2ss, and always update if it wasn't us last
void I2S::aquire() {
     if (_owner != this) {
    	i2s_format(&_i2s, _dbits, _fbits, _polarity);
        i2s_audio_frequency(&_i2s, _hz);
        i2s_set_protocol(&_i2s, _protocol);
        i2s_set_mode(&_i2s, _mode);
        _owner = this;
    }
}

int I2S::transfer(const I2S::I2STransferAdder &td)
{
    bool queue;
    {
        CriticalSectionLock lock;
        queue = _busy;
        _busy = true;
    }
    if (queue || i2s_active(&_i2s)) {
        return queue_transfer(td._td);
    }
    start_transfer(td._td);
    return 0;
}

void I2S::abort_transfer()
{
    i2s_abort_asynch(&_i2s);
#if TRANSACTION_QUEUE_SIZE_I2S
    dequeue_transaction();
#endif
}


void I2S::clear_transfer_buffer()
{
#if TRANSACTION_QUEUE_SIZE_I2S
    _transaction_buffer.reset();
#endif
}

void I2S::abort_all_transfers()
{
    clear_transfer_buffer();
    abort_transfer();
}

int I2S::get_transfer_status()
{
    if (i2s_active(&_i2s)) {
        return -1;
    }
    return  0;
}

unsigned int I2S::get_module()
{
	return i2s_get_module(&_i2s);
}

int I2S::queue_transfer(const transaction_data_t &td)
{
#if TRANSACTION_QUEUE_SIZE_I2S
    CriticalSectionLock lock;
    int result;

    transaction_t transaction(this, td);
    if (_transaction_buffer.full()) {
        result = -1;
    } else {
        _transaction_buffer.push(transaction);
        result = 0;
    }
    return result;
#else
    return -1;
#endif
}

void I2S::start_transfer(const transaction_data_t &td)
{
    aquire();
    _current_transaction = td;
    _irq_tx.callback(&I2S::irq_handler_asynch_tx);
    _irq_rx.callback(&I2S::irq_handler_asynch_rx);
    i2s_master_transfer(&_i2s,
    		td._transaction.tx_buffer.buf, td._transaction.tx_buffer.length,
			td._transaction.rx_buffer.buf, td._transaction.rx_buffer.length,
			td._circular,
            _irq_tx.entry(), _irq_rx.entry(), td._transaction.event);
}

#if TRANSACTION_QUEUE_SIZE_I2S

void I2S::start_transaction(transaction_data_t *data)
{
    start_transfer(*data);
}

void I2S::dequeue_transaction()
{
    transaction_t t;
    bool dequeued;
    {
        CriticalSectionLock lock;
        dequeued = _transaction_buffer.pop(t);
        _busy = dequeued;
    }

    if (dequeued) {
        I2S* obj = t.get_object();
        transaction_data_t* data = t.get_transaction();
        obj->start_transaction(data);
    }
}

#endif // TRANSACTION_QUEUE_SIZE_I2S

void I2S::irq_handler_asynch_rx(void)
{
    int event = i2s_irq_handler_asynch(&_i2s, I2S_RX_EVENT);
    if (_current_transaction._transaction.callback && (event & I2S_EVENT_ALL)) {
        minar::Scheduler::postCallback(
                _current_transaction._transaction.callback.bind(_current_transaction._transaction.tx_buffer,
                		_current_transaction._transaction.rx_buffer,
                        event & I2S_EVENT_ALL));
    }
#if TRANSACTION_QUEUE_SIZE_I2S
    if (event & (I2S_EVENT_ALL | I2S_EVENT_INTERNAL_TRANSFER_COMPLETE)) {
        dequeue_transaction();
    }
#endif
}

void I2S::irq_handler_asynch_tx(void)
{
	// betzw - TODO
    int event = i2s_irq_handler_asynch(&_i2s, I2S_TX_EVENT);
    if (_current_transaction._transaction.callback && (event & I2S_EVENT_ALL)) {
        minar::Scheduler::postCallback(
                _current_transaction._transaction.callback.bind(_current_transaction._transaction.tx_buffer,
                		_current_transaction._transaction.rx_buffer,
                        event & I2S_EVENT_ALL));
    }
#if TRANSACTION_QUEUE_SIZE_I2S
    if (event & (I2S_EVENT_ALL | I2S_EVENT_INTERNAL_TRANSFER_COMPLETE)) {
        dequeue_transaction();
    }
#endif
}

I2S::I2STransferAdder::I2STransferAdder(I2S *owner) :
        _applied(false), _rc(0), _owner(owner)
{
    _td._transaction.tx_buffer.length = 0;
    _td._transaction.rx_buffer.length = 0;
    _td._transaction.callback = event_callback_t((void (*)(Buffer, Buffer, int))NULL);
    _td._circular = false;
}
const I2S::I2STransferAdder & I2S::I2STransferAdder::operator =(const I2S::I2STransferAdder &a)
{
    _td = a._td;
    _owner = a._owner;
    _applied = 0;
    return *this;
}
I2S::I2STransferAdder::I2STransferAdder(const I2STransferAdder &a)
{
    *this = a;
}
I2S::I2STransferAdder & I2S::I2STransferAdder::tx(void *txBuf, size_t txSize)
{
    MBED_ASSERT(!_td._transaction.tx_buffer.length);
    _td._transaction.tx_buffer.buf = txBuf;
    _td._transaction.tx_buffer.length = txSize;
    return *this;
}
I2S::I2STransferAdder & I2S::I2STransferAdder::rx(void *rxBuf, size_t rxSize)
{
    MBED_ASSERT(!_td._transaction.rx_buffer.length);
    _td._transaction.rx_buffer.buf = rxBuf;
    _td._transaction.rx_buffer.length = rxSize;
    return *this;
}
I2S::I2STransferAdder & I2S::I2STransferAdder::circular(bool mode)
{
	_td._circular = mode;
	return *this;
}
I2S::I2STransferAdder & I2S::I2STransferAdder::callback(const event_callback_t &cb, int event)
{
    MBED_ASSERT(!_td._transaction.callback);
    _td._transaction.callback = cb;
    _td._transaction.event = event;
    return *this;
}
int I2S::I2STransferAdder::apply()
{
    if (!_applied) {
        _applied = true;
        _rc = _owner->transfer(*this);
    }
    return _rc;
}
I2S::I2STransferAdder::~I2STransferAdder()
{
    apply();
}

I2S::I2STransferAdder I2S::transfer()
{
    I2STransferAdder a(this);
    return a;
}

} // namespace mbed

#endif
