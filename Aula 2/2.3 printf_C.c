#include <stdio.h>

int main()
{
    int i;
    int primos[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71};
    for(i=0;i<sizeof(primos)/sizeof(primos[0]);i++)
    {
        printf("Primo %i = %i\n",i+1,primos[i]);
    }

	return 0;
}