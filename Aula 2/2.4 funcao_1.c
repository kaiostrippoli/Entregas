#include <stdio.h>

int MDC(int n1, int n2)
{
    int i,mdc = 1;
    for(i=1;i<=n1||i<=n2;++i)
    {
        if(n1%i==0&&n2%i==0)
            mdc=i;
    }
    return mdc;
}

int main()
{
	int num1,num2,hcf;
    printf("Numero1:\n");
    scanf("%d",&num1);
    printf("Numero2:\n");
    scanf("%d",&num2);
    hcf = MDC(num1,num2);
    printf("H.C.F of %d and %d is %d\n",num1,num2,hcf);
	return 0;
}