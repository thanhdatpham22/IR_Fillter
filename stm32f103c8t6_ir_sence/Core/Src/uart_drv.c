#include "uart_drv.h"
#include "drv_adc.h"
uint8_t datatx[]="Gia tri adc la: ";
uint8_t data[6]={};

// Hàm chuyển đổi từ uint16_t sang chuỗi ký tự
void itoacode(uint16_t value, uint8_t *buffer, int radix) {
    int i = 0;
    int isNegative = 0;
    int is10 = 0;

    // Xử lý trường hợp giá trị âm (chỉ áp dụng cho radix = 10)
    if (value < 0 && radix == 10) {
        isNegative = 1;
        value = -value;
    }
    if (value < 10 ) {
    	is10 = 1;//nho hon 10
    }
    // Chuyển đổi giá trị thành chuỗi ngược
    do {
        int digit = value % radix;
        buffer[i++] = (digit < 10) ? (digit + '0') : (digit - 10 + 'A');
        value /= radix;
    } while (value > 0);

    // Nếu là giá trị âm và radix là 10, thêm dấu âm
    if (isNegative && radix == 10) {
        buffer[i++] = '-';
    }
    if(is10==1)
    {
    	buffer[i++]='0';
    }
    // Đảo chuỗi để có giá trị đúng
    for (int j = 0; j < i / 2; j++) {
        char temp = buffer[j];
        buffer[j] = buffer[i - j - 1];
        buffer[i - j - 1] = temp;
    }

    // Kết thúc chuỗi
    buffer[i] = '\0';
}
void reset_buffer(uint8_t * str,uint8_t size)
{
	for(int i=0;i<size;i++)
	{
		*(str+i)='\0';
	}
}
void Task_uart()
{
	HAL_UART_Transmit(&huart3,datatx,sizeof(datatx),100);

	uint8_t c='-';
	uint8_t line[]="\n\r";

	reset_buffer(data,sizeof(data));
	itoacode((uint16_t)Adc_getvalue(0),data,10);

	HAL_UART_Transmit(&huart3,data,sizeof(data),100);
	HAL_UART_Transmit(&huart3,&c,1,100);

	reset_buffer(data,sizeof(data));
	itoacode((uint16_t)Adc_getvalue(1),data,10);

	HAL_UART_Transmit(&huart3,data,sizeof(data),100);
	HAL_UART_Transmit(&huart3,&c,1,100);

	reset_buffer(data,sizeof(data));
	itoacode((uint16_t)Adc_getvalue(2),data,10);

	HAL_UART_Transmit(&huart3,data,sizeof(data),100);
	HAL_UART_Transmit(&huart3,&c,1,100);

	reset_buffer(data,sizeof(data));
	itoacode((uint16_t)Adc_getvalue(3),data,10);

	HAL_UART_Transmit(&huart3,data,sizeof(data),100);


	HAL_UART_Transmit(&huart3,line,sizeof(line),100);
}
