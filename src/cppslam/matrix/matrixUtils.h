/* Header file for Matrix utilities helper file. Utilises the matrix library for some more complex algorithms */
#include "matrix.h"
#include "ros/ros.h"
#include <string.h>
#include <iostream>

void matrixSqrt(Matrix &U);
void schurDecomposition(Matrix &U, Matrix &Q, Matrix &R);
void QRDecomposition(Matrix &A, Matrix &Q, Matrix &R);
