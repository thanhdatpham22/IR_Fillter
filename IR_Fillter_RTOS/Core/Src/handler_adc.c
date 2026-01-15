/*
 * handler_adc.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */
#include "handler_adc.h"

void ADC1_Calibration(void)
{
    // Bước 1: Đảm bảo ADC đã được kích hoạt Clock (Phải được thực hiện ở thanh ghi RCC trước đó)
    // Giả định: Clock cho ADC1 đã được bật (RCC_APB2ENR |= RCC_APB2ENR_ADC1EN)

    // Bước 2: Bật ADC (Thiết lập bit ADON)
    // Cần bật ADC trước khi hiệu chuẩn.
    ADC1->CR2 |= ADC_CR2_ADON;

    // Chờ một thời gian ngắn (tối thiểu 2 chu kỳ clock ADC) để ADC ổn định (thao tác này thường đã đủ).

    // Bước 3: Thiết lập bit RSTCAL để reset giá trị hiệu chuẩn trước đó
    ADC1->CR2 |= ADC_CR2_RSTCAL;

    // Bước 4: Chờ cho bit RSTCAL tự động xóa (báo hiệu việc reset đã hoàn tất)
    // Vòng lặp chờ: Đợi cho thanh ghi tự động xóa bit RSTCAL
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0);

    // Bước 5: Thiết lập bit CAL để bắt đầu quá trình tự hiệu chuẩn
    ADC1->CR2 |= ADC_CR2_CAL;

    // Bước 6: Chờ cho bit CAL tự động xóa (báo hiệu quá trình hiệu chuẩn đã hoàn tất)
    // Vòng lặp chờ: Đợi cho thanh ghi tự động xóa bit CAL
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0);

    // Quá trình hiệu chuẩn ADC1 đã hoàn thành.
    // Mã hiệu chỉnh đã được tính toán và lưu nội bộ.
}

int compare(const void *a, const void *b)
{
    return (*(int*)a - *(int*)b);
}
/* Hoán đổi hai phần tử uint16_t */
static void swap_uint16(uint16_t *a, uint16_t *b)
{
    uint16_t temp = *a;
    *a = *b;
    *b = temp;
}

/* Hàm phân vùng cho uint16_t */
static int partition_uint16(uint16_t arr[], int low, int high)
{
    uint16_t pivot = arr[high];
    int i = low - 1;

    for (int j = low; j < high; j++)
    {
        if (arr[j] <= pivot)
        {
            i++;
            swap_uint16(&arr[i], &arr[j]);
        }
    }
    swap_uint16(&arr[i + 1], &arr[high]);
    return i + 1;
}

/* Hàm QuickSort đệ quy cho uint16_t */
static void quickSort_uint16(uint16_t arr[], int low, int high)
{
    if (low < high)
    {
        int pi = partition_uint16(arr, low, high);

        quickSort_uint16(arr, low, pi - 1);
        quickSort_uint16(arr, pi + 1, high);
    }
}

/* Hàm tiện ích để gọi từ bên ngoài */
void quickSortArray_uint16(uint16_t arr[], int n)
{
    quickSort_uint16(arr, 0, n - 1);
}
uint16_t Average_Caculate(uint16_t * adc_array, uint8_t size, uint8_t skip)
{
    if (size <= 0 || skip < 0 || 2 * skip >= size) {
        return 0;
    }

    uint8_t start = skip;
    uint8_t end = size - skip - 1;
    uint8_t count = end - start + 1;
    uint32_t tong = 0;

    for (uint8_t i = start; i <= end; i++)
    {
        tong += adc_array[i];
    }

    return tong / count;
}


uint16_t MedianAverage(uint16_t *adc_array, uint8_t size)
{
    if (size == 0) return 0;

    // Bước 1: Tìm min và max
    uint16_t min = adc_array[0];
    uint16_t max = adc_array[0];
    for (uint8_t i = 1; i < size; i++)
    {
        if (adc_array[i] < min) min = adc_array[i];
        if (adc_array[i] > max) max = adc_array[i];
    }

    // Bước 2: Tính median approx
    uint16_t median = (min + max) / 2;

    // Bước 3: Xác định range ±X% quanh median
    uint16_t lower = median - (median * SKIP_PERCENT / 100);
    uint16_t upper = median + (median * SKIP_PERCENT / 100);

    // Bước 4: Duyệt mảng, cộng các giá trị nằm trong range
    uint32_t sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < size; i++)
    {
        if (adc_array[i] >= lower && adc_array[i] <= upper)
        {
            sum += adc_array[i];
            count++;
        }
    }

    // Bước 5: Trả về giá trị trung bình
    if (count == 0) return median; // fallback
    return (uint16_t)(sum / count);
}
