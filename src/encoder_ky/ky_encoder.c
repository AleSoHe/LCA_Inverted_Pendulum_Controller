// Variables
#include "Pin_ky_A.h"
#include "Pin_ky_B.h"

int A;
int B;
int lastA;
int pos = 0;

void init_ky(){
    lastA = Pin_A_Read();  
}

// Functions
int get_ky_position(void)
{   
    A = Pin_A_Read();
    B = Pin_B_Read();
    
    if(lastA != A){
        if(A != B){
            pos++;
        }
        else{
            pos--;
        }
        lastA = A;
    }
    return pos;
}
