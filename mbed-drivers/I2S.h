#ifndef MBED_I2S_H
#define MBED_I2S_H

#include "platform.h"

#if DEVICE_I2S

#include "i2s_api.h"

#include "CThunk.h"
#include "dma_api.h"
#include "CircularBuffer.h"
#include "core-util/FunctionPointer.h"
#include "Transaction.h"

namespace mbed {

/** A I2S Master/Slave, used for communicating with I2S slave devices
 *
 * The default format is set to master transmission mode, 16 data bits & 16 bits per frame,
 * clock polarity 0, protocol PCM_SHORT, and a clock frequency of 44.1kHz
 *
 * NOTE: This information will be deprecated soon.
 * Most I2S devices will also require Reset signals. These
 * can be controlled using <DigitalOut> pins
 */
class I2S {

public:
    /** I2S transfer callback
     *  @param Buffer the tx buffer
     *  @param Buffer the rx buffer
     *  @param int the event that triggered the callback
     */
    typedef mbed::util::FunctionPointer3<void, Buffer, Buffer, int> event_callback_t;
private:
    typedef struct {
    	TwoWayTransaction<event_callback_t> _transaction;
    	bool _circular;
    } transaction_data_t;
    typedef Transaction<I2S, transaction_data_t> transaction_t;

public:
    /** Create a I2S master connected to the specified pins
     *
     *  @param dpin  I2S data input/output pin
     *  @param clk   I2S Clock output pin
     *  @param wsel  I2S word select output pin (might be NC for PDM sources)
     *  @param fdpin I2S data input pin (for full-duplex operation, default = NC)
     */
    I2S(PinName dpin, PinName clk, PinName wsel, PinName fdpin = NC);

    /** Configure the data transmission format
     *
     *  @param dbits Number of data bits per I2S frame (16, 24, or 32)
     *  @param fbits Number of bits per I2S frame (16 or 32)
     *  @param polarity Clock polarity (either 0/low or 1/high, default = 0)
     */
    void format(int dbits, int fbits, int polarity = 0);

    /** Set the i2s audio frequency
     *
     *  @param hz audio frequency in hz
     */
    void audio_frequency(unsigned int hz);

    /** Set the i2s bus protocol
     *
     *  @param protocol I2S protocol to be used
     *  @return Zero if the usage was set, -1 if a transaction is on-going
     */
    void set_protocol(i2s_bitorder_t protocol);

    /** Set the i2s mode
     *
     *  @param mode I2S mode to be used
     *  @return Zero if the usage was set, -1 if a transaction is on-going
     */
    void set_mode(i2s_mode_t mode);

    class I2STransferAdder {
        friend I2S;
    private:
        I2STransferAdder(I2S *owner);
        const I2STransferAdder & operator =(const I2STransferAdder &a);
        I2STransferAdder(const I2STransferAdder &a);
    public:
        /** Set the transmit buffer
         *  Sets the transmit buffer pointer and transmit size.
         *
         *  NOTE: Repeated calls to tx() override buffer parameters.
         *
         *  @param[in] txBuf a pointer to the transmit buffer
         *  @param[in] txSize the size of the transmit buffer
         *  @return a reference to the I2STransferAdder
     */
        I2STransferAdder & tx(void *txBuf, size_t txSize);
        /** Set the receive buffer
         *  Sets the receive buffer pointer and receive size
     *
         *  NOTE: Repeated calls to rx() override buffer parameters.
         *
         *  @param[in] rxBuf a pointer to the receive buffer
         *  @param[in] rxSize the size of the receive buffer
         *  @return a reference to the I2STransferAdder
     */
        I2STransferAdder & rx(void *rxBuf, size_t rxSize);
        /** Set the I2S Event callback
         *  Sets the callback to invoke when an event occurs and the mask of
         *  which events should trigger it. The callback will be scheduled to
         *  execute in main context, not invoked in interrupt context.
     *
         *  NOTE: Repeated calls to callback() override callback parameters.
         *
         *  @param[in] cb The event callback function
         *  @param[in] event     The logical OR of I2S events to modify. Look at i2s hal header file for I2S events.
         *  @return a reference to the I2STransferAdder
         */

        // betzw - TODO: circularity of transfer
        I2STransferAdder & circular(bool);

        I2STransferAdder & callback(const event_callback_t &cb, int event);
        /** Initiate the transfer
         *  apply() allows the user to explicitly activate the transfer and obtain
         *  the return code from the validation of the transfer parameters.
         * @return Zero if the transfer has started, or -1 if I2S peripheral is busy
         */
        int apply();
        ~I2STransferAdder();
    private:
        transaction_data_t _td;
        bool _applied;
        int _rc;
        I2S * _owner;
    };
    /** Start an I2S transfer
     *  The transfer() method returns a I2STransferAdder.  This class allows each
     *  parameter to be set with a dedicated method.  This way, the many optional
     *  parameters are easy to identify and set.
     *
     *
     *  @return A I2STransferAdder object.  When either apply() is called or the
     *      I2STransferAdder goes out of scope, the transfer is queued.
     */
    I2STransferAdder transfer();

    /** Abort the on-going I2S transfer, and continue with transfer's in the queue if any.
     */
    void abort_transfer();

    /** Clear the transaction buffer
     */
    void clear_transfer_buffer();

    /** Clear the transaction buffer and abort on-going transfer.
     */
    void abort_all_transfers();

    /** Get transfer status
     *
     *  @return -1 if a transaction is on-going, zero otherwise
    */
    int get_transfer_status();

protected:
    /** I2S TX DMA IRQ handler
     *
    */
    void irq_handler_asynch_tx(void);

    /** I2S RX DMA IRQ handler
     *
    */
    void irq_handler_asynch_rx(void);

    /** Add a transfer to the queue
     * @param data Transaction data
     * @return Zero if a transfer was added to the queue, or -1 if the queue is full
    */
    int queue_transfer(const transaction_data_t &td);

    /** Configures a callback, i2s peripheral and initiate a new transfer
     *
     * @param data Transaction data
    */
    void start_transfer(const transaction_data_t &td);

#if TRANSACTION_QUEUE_SIZE_I2S
    /** Start a new transaction
     *
     *  @param data Transaction data
    */
    void start_transaction(transaction_data_t *data);

    /** Dequeue a transaction
     *
    */
    void dequeue_transaction();
#endif // TRANSACTION_QUEUE_SIZE_I2S

    /** Initiate a transfer
     * @param xfer the I2STransferAdder object used to create the I2S transfer
     * @return the result of validating the transfer parameters
     */
    int transfer(const I2STransferAdder &xfer);

public:
    virtual ~I2S() {
    }

protected:
    i2s_t _i2s;

#if TRANSACTION_QUEUE_SIZE_I2S
    static CircularBuffer<transaction_t, TRANSACTION_QUEUE_SIZE_I2S> _transaction_buffer;
#endif
    CThunk<I2S> _irq_tx;
    CThunk<I2S> _irq_rx;
    transaction_data_t _current_transaction;

    void aquire(void);
    static I2S *_owner;
    int _dbits;
    int _fbits;
    int _polarity;
    i2s_bitorder_t _protocol;
    i2s_mode_t _mode;
    bool _busy;
    unsigned int _hz;
};

} // namespace mbed

#endif

#endif
