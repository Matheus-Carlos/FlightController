#include "average.h"




average::average(int _n_points){
    n_points = _n_points;
    
}

float average::average_compute(float measurement){

    float numbers[n_points];
//desloca os elementos do vetor de média móvel
    for(int i= n_points-1; i>0; i--) numbers[i] = numbers[i-1];

    numbers[0] = measurement; //posição inicial do vetor recebe a leitura original

    float acc = 0;          //acumulador para somar os pontos da média móvel

    for(int i=0; i<n_points; i++) acc += numbers[i]; //faz a somatória do número de pontos


    return acc/n_points;  //retorna a média móvel
}