#include <stdio.h>

void print_array(int *arr, int length)
{
    int i;
    for(i=0;i<length;i++)
    {
        printf("test_array[%i] = %i\n",i+1,*(arr+i));
    }
}

int main()
{
    int test_array[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
    print_array(&test_array, sizeof(test_array)/sizeof(test_array[0]));
	return 0;
}