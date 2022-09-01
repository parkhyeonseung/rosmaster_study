#include <stdio.h>
#include <iostream>
using namespace std;
int main() {
    char temp[6] ;
    int val;
    char tt[5];
    temp[0] = 'q';
    temp[1] = '2';
    temp[2] = '3';
    for( int i = 1 ; i < 6 ; i ++ ){
		tt[i-1] = temp[i]; 
	}
	cout << endl;
    val = atoi(tt);
    cout << val;
    cout << endl;
}