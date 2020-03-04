/*
Matrix utilities helper file.
Utilises the matrix library for some more complex algorithms
Author: Jack McRobbie
*/
#include "matrixUtils.h"

void matrixSqrt(Matrix &A)
{
    /* 
    Uses matrix decomposition via schur decomposition to determine the 
    matrix sqrt.
    U^1/2 = Q^T R^1/2 Q
    */
   try{
    if (A.IsSquare())
    {
	    int dim = A.GetNRows();
	    Matrix T(dim,dim);
	    Matrix U(dim,dim);
            
            U.Fill(0.0);
            T.Fill(0.0);
           
	    schurDecomposition(A, T, U);
	    T.PrintMatrix();
	   // T.ElementSqrt();

	    T.PrintMatrix();
	    
            Matrix Rs(dim,dim);
	    Matrix tmp(dim,dim);
	    Matrix Ut(dim,dim);
	    Rs.Fill(0.0);
	    tmp.Fill(0.0);
	    Ut.Fill(0.0);
	    Ut.Transpose(U);
	    tmp.Multiply(T,Ut);
	    Matrix result(dim,dim);
	    result.Fill(0.0);
 	    result.Multiply(U,tmp);
	    A.CopyMatrix(result);
	    A.PrintMatrix();	
            T.Multiply(U,Ut);
	   T.PrintMatrix(); 
    }

    else
    {
        throw "Matrix Not square!";
    }
   }
   catch(const char* msg)
   {
       ROS_ERROR_STREAM(msg);
   }
}
void schurDecomposition(Matrix &A, Matrix &T, Matrix &U)
/*
 Decomposes the matrix U as follows 
 A = U^-1 T U 
recovers R & Q
*/
{
    int dim = U.GetNRows();
    Matrix Ut(dim,dim);
    Matrix At(dim,dim);
    Matrix Q(dim,dim);
    Matrix R(dim,dim);

    Ut.Identity();
    At.CopyMatrix(A);
    for (int i = 0; i <100; i++)
    {
        QRDecomposition(At, Q, R);
        At.Multiply(R, Q);
	Ut.Multiply(Ut,Q);
    }
    ROS_INFO_STREAM("RESULT OF SCHUR DECOMPOSITION");
    T.CopyMatrix(At);
    U.CopyMatrix(Ut);
    return;
}
void QRDecomposition(Matrix &A, Matrix &Q, Matrix &R)
{
    /* 
   Acheives A = QR decomposition via the Jacobi method
   TODO: Make this less messy by accessing parent class via pointer
    rather than copying a new one. 
   */
    double sum;
    int dim = A.GetNRows();
    Matrix U(A.GetNRows(),A.GetNColumns());
    U.GrammSchmidtNorm(A);
    for (int i = 0; i < A.GetNColumns(); i++)
    {
        sum = 0.0;

        for (int j = 0; j < A.GetNRows(); j++)
        {
            sum += U.Get(j, i) * U.Get(j, i);
        }
        sum = sqrt(sum);
        for (int j = 0; j <U.GetNRows(); j++)
        {
            Q.Set(j, i, U.Get(j, i) / sum);
        }

    }
    Matrix Qt(dim,dim);
    Qt.Transpose(Q);
    R.Multiply(Qt, A);
    return;
}
