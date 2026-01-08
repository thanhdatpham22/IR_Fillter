/*
 * handler_adc.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */
#include "handler_adc.h"

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
