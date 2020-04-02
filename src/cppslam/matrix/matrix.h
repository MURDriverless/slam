#ifndef MATRIX_H
#define MATRIX_H

/*
Lightweight Matrix library for use with the automous stack
Author(s): Jack McRobbie, Rowan Skewes, Robert Chin 

TODO: EXCEPTION throwing on incorrect dimensions!

*/
#include <cstddef>
#include "ros/ros.h"
#include <string.h>
#include <iostream>
#include <math.h>

#define NewStackMatrixMacro(name, n_rows, n_cols) \
    double name##_data[n_rows][n_cols];           \
    Matrix name(name##_data);

class Matrix
{
public:
    template <int rows, int columns>
    explicit Matrix(double (&data)[rows][columns])
        : data(&data[0][0]), nrows(rows), ncolumns(columns) {}

    // To instantiate a constant matrix, use this constructor (constant data),
    // a dummy 2D array of mutable data, but declare the object as 'const'
    // to disallow any modification of the inner array.
    template <int rows, int columns>
    explicit Matrix(const double (&const_data)[rows][columns],
                    double (&data)[rows][columns])
        : data(&data[0][0]), nrows(rows), ncolumns(columns)
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                data[i][j] = const_data[i][j];
            }
        }
    }

    Matrix(int rows, int columns)
    {
        nrows = rows;
        ncolumns = columns;
        double *init_data = new double[rows * columns - 1];
        data = init_data;
        Fill(0.0);
    }

    template <int rows, int columns>
    Matrix(const Matrix &A, double (&init_data)[rows][columns] = NULL)
    {
        nrows = A.GetNRows();
        ncolumns = A.GetNColumns();
        data = &init_data[0][0];
        for (int i = 0; i < nrows; i++)
        {
            for (int j = 0; j < ncolumns; j++)
            {
                data[(i * ncolumns) + j] = A.Get(i, j);
            }
        }
    }

    void CopySlice(int row_start, int row_end, int column_start,
                   int column_end, const Matrix &A);

    int GetNRows() const;
    int GetNColumns() const;
    bool IsSquare() const;

    double Get(int row, int column) const;
    void Set(int row, int column, double value);

    static bool DoubleIsEqual(double a, double b);
    bool IsEqual(const Matrix &A) const;
    bool SameSize(const Matrix &A) const;
    bool SameNRows(const Matrix &A) const;
    bool SameNColumns(const Matrix &A) const;

    void Transpose(const Matrix &A);
    static double VectorNorm(const Matrix &A);

    void Add(const Matrix &A, const Matrix &B);
    void Subtract(const Matrix &A, const Matrix &B);
    void Multiply(const Matrix &A, const Matrix &B);
    void MultiplyScalar(const Matrix &A, double scale);
    void CrossProduct(const Matrix &A, const Matrix &B);
    static double DotProduct(const Matrix &A, const Matrix &B);
    void CopyMatrix(Matrix &A);
    void Fill(double value);
    void CopyInto(int row_start, int column_start, const Matrix &A);
    void Identity();

    void SkewSymmetricFill(const Matrix &V);

    void ConcatenateHorizontally(const Matrix &A, const Matrix &B);
    void ConcatenateVertically(const Matrix &A, const Matrix &B);
    void ElementSqrt();
    void AddRows(int row_to, int row_from, double scale);
    void MultiplyRow(int row, double scale);
    void SwitchRows(int row_a, int row_b);
    void RowReduce();
    void PrintMatrix();
    void GrammSchmidtNorm(Matrix &A);
    void matrixSqrt(Matrix &U);

    template <int size>
    void InvertMatrix(const Matrix &A)
    {
        NewStackMatrixMacro(augmented, size, 2 * size);
        augmented.CopyInto(0, 0, A);

        NewStackMatrixMacro(identity, size, size);
        identity.Identity();
        augmented.CopyInto(0, size, identity);

        augmented.RowReduce();

        CopySlice(0, size - 1, size, 2 * size - 1, augmented);
    }

    void QuaternionNormalise(const Matrix &q);
    void RotationMatrixFromQuaternion(const Matrix &q);
    void QuaternionConjugate();
    void QuaternionProductCross(Matrix &a, Matrix &b);
    void QuaternionProductDot(Matrix &a, Matrix &b);
    void projection(Matrix &u, Matrix &v);

private:
    //complex *comp_data;
    double *data;
    int nrows;
    int ncolumns;
    static constexpr double EPSILON_MULT = 1E-6; //  Comparison ratio
    static constexpr double EPSILON_ADD = 1E-4;  //  Comparison ratio
};

#endif
