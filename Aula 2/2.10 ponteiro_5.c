#include <stdio.h>
#include <stlib.h>

int *primos(void){
    int *v;
    v= malloc(3);
    v[0] = 1009; v[1] = 1013; v[2] = 1019;
    return v;
}

int main()
{
    int i;
    int *arr;
    arr = primos();
    free(primos());
    for(i=0;i<3;i++)
    {
        printf("test_array[%i] = %i\n",i+1,*(arr+i));
    }
	return 0;
}