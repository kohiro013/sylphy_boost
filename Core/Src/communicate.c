
#include "defines.h"
#include "global.h"

#define _BS				0x08	// バックスペース
#define _ESC			0x1b	// エスケープシーケンス
#define TRX_BUFFER_SIZE 127		// 送受信バッファサイズ

struct {
    volatile int16_t 	head;
    volatile int16_t	tail;
    volatile int16_t	remain;
    uint8_t	data[TRX_BUFFER_SIZE];
} tx_buffer = {0}, rx_buffer = {0}; // FIFOバッファ


/* ---------------------------------------------------------------
	UART1で1文字受信する関数
--------------------------------------------------------------- */
uint8_t Communicate_Receive1byte(void)
{
	uint8_t data = '\0';
	while(LL_USART_IsActiveFlag_RXNE(USART1) == RESET);
	data = LL_USART_ReceiveData8(USART1);
	LL_USART_ClearFlag_RXNE(USART1);
	return data;
}

uint8_t Communicate_ReceiceDMA(void)
{
// head（DMACが受信データを書き込む位置）に新しく受信データが蓄積される．
// tailから読み出せばいい
// headがtailに追いつくとバッファオーバーフローとなり正しく読み出せない

	// この関数は多重に実行されるとまずいので割り込みを禁止する
	__disable_irq();

	// headの位置を読み取る
	rx_buffer.head = TRX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5);

	// バッファが空ならヌル文字を返す
	if(rx_buffer.head == rx_buffer.tail){
		// 割り込み許可
		__enable_irq();

		return '\0';
	} else;

	// バッファから一文字読み取り返す
	char ch;
	ch = rx_buffer.data[rx_buffer.tail++];
	if(rx_buffer.tail == TRX_BUFFER_SIZE) rx_buffer.tail = 0;

	// 割り込み許可
	__enable_irq();

	return ch;
}

/* ---------------------------------------------------------------
	受信バッファを空にする関数
--------------------------------------------------------------- */
void Communicate_ClearReceiveBuffer(void)
{
	while(Communicate_ReceiceDMA() != '\0');
}

/* ---------------------------------------------------------------
	UART1で1文字送信する関数
--------------------------------------------------------------- */
void Communicate_Transmit1byte(uint8_t ch)
{
	LL_USART_TransmitData8(USART1, ch);
	while(LL_USART_IsActiveFlag_TXE(USART1) == 0);
}

void Communicate_TransmitDMA(uint8_t ch)
{
// headに新しく追加する
// tailは次に送信されるデータを指す
// バッファに空きが無い（headがtailに追いついた）場合は待機する

	// バッファ内データ数をカウントし，空きがない場合待機する
	// バッファフルで待機しているときには割り込みを許可するためにwhileループになっている
	while(1) {
		// この関数は多重に実行されるとまずいので割り込みを禁止する
		__disable_irq();

		// DMACを一時的に停止
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
		while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7) != DISABLE);

		// 転送データ残数を取得
		tx_buffer.remain = LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_7);

		// バッファに空きがあればループから抜ける
		if(tx_buffer.remain != TRX_BUFFER_SIZE) break;

		// DMAC動作再開
		LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)tx_buffer.data,
			(uint32_t)LL_USART_DMA_GetRegAddr(USART1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_7));
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, TRX_BUFFER_SIZE);
		LL_DMA_ClearFlag_TC7(DMA2);
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);

		// 割り込み許可
		__enable_irq();

		// バッファに空きができるまで待機（この間割り込みが発生してもよい）
		while(LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_7) == TRX_BUFFER_SIZE);
	}
	// ここの時点でDMACは停止，割り込みは禁止されている

	// 次に書き込もうとする領域がバッファ終端なら前に詰める
	if(tx_buffer.head == TRX_BUFFER_SIZE){
		int blank = TRX_BUFFER_SIZE - tx_buffer.remain; // バッファの空き数
		int i;
		for(i=0;i+blank<TRX_BUFFER_SIZE;i++){
			tx_buffer.data[i] = tx_buffer.data[i+blank];
		}
		tx_buffer.head = i;
	}

	// 1文字追加
	tx_buffer.data[tx_buffer.head++] = ch;
	tx_buffer.remain++;
	tx_buffer.tail = tx_buffer.head - tx_buffer.remain;

	// DMACの再設定
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)(&tx_buffer.data[tx_buffer.tail]),
		(uint32_t)LL_USART_DMA_GetRegAddr(USART1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_7));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, tx_buffer.remain);
	LL_DMA_ClearFlag_TC7(DMA2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);

	// 割り込み許可
	__enable_irq();
}

/* ---------------------------------------------------------------
	printfとscanfを使用するための設定
--------------------------------------------------------------- */
void Communicate_Initialize( void )
{
	// USARTによるDMAリクエストon
	LL_USART_EnableDMAReq_TX(USART1);
	LL_USART_EnableDMAReq_RX(USART1);

	// 受信DMA動作開始
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_5, (uint32_t)LL_USART_DMA_GetRegAddr(USART1),
		(uint32_t)rx_buffer.data, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_5));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, TRX_BUFFER_SIZE);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);

	// エラー時の割り込みを禁止
	LL_USART_DisableIT_PE(USART1);
	LL_USART_DisableIT_ERROR(USART1);

	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
}

/* ---------------------------------------------------------------
	printfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	//Communicate_Transmit1byte(ch);
	Communicate_TransmitDMA(ch);
	return 1;
}

/* ---------------------------------------------------------------
	scanfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
GETCHAR_PROTOTYPE
{
//	return Communicate_Receive1byte();
	volatile uint8_t ch;
	while((ch = Communicate_ReceiceDMA()) == '\0');
	return ch;
}
