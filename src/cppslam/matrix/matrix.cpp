#include <math.h>
#include "matrix.h"

/*
Lightweight Matrix library for use with the automous stack
Author(s): Jack McRobbie, Rowan Skewes, Robert Chin 

TODO: EXCEPTION throwing on incorrect dimensions!
*/

void Matrix::CopySlice(int row_start, int row_end, int column_start,
                       int column_end, const Matrix &A)
{

    nrows = row_end - row_start + 1;
    ncolumns = column_end - column_start + 1;
    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            Set(i, j, A.Get(i + row_start, j + column_start));
        }
    }
}

int Matrix::GetNRows() const { return nrows; }

int Matrix::GetNColumns() const { return ncolumns; }

bool Matrix::IsSquare() const { return nrows == ncolumns; }

double Matrix::Get(int row, int column) const
{

    return data[(row * ncolumns) + column];
}

void Matrix::Set(int row, int column, double value)
{

    data[(row * ncolumns) + column] = value;
}

//  Test equality using relative difference. Additive tolerance allows
//  comparison with zero, but it's set arbitrarily.
bool Matrix::DoubleIsEqual(double a, double b)
{

    if (a > b)
    {
        return fabs(a - b) <= fabs(a * EPSILON_MULT + EPSILON_ADD);
    }
    return fabs(a - b) <= fabs(b * EPSILON_MULT + EPSILON_ADD);
}

bool Matrix::IsEqual(const Matrix &A) const
{

    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            if (!DoubleIsEqual(Get(i, j), A.Get(i, j)))
            {
                return false;
            }
        }
    }
    return true;
}

bool Matrix::SameSize(const Matrix &A) const
{
    return SameNRows(A) && SameNColumns(A);
}

bool Matrix::SameNRows(const Matrix &A) const { return nrows == A.GetNRows(); }

bool Matrix::SameNColumns(const Matrix &A) const
{
    return ncolumns == A.GetNColumns();
}

void Matrix::Transpose(const Matrix &A)
{

    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            Set(i, j, A.Get(j, i));
        }
    }
}

double Matrix::VectorNorm(const Matrix &A)
{

    double sum_of_squares = 0;
    for (int i = 0; i < A.GetNRows(); i++)
    {
        sum_of_squares += A.Get(i, 0) * A.Get(i, 0);
    }
    return sqrt(sum_of_squares);
}

void Matrix::Add(const Matrix &A, const Matrix &B)
{

    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            Set(i, j, A.Get(i, j) + B.Get(i, j));
        }
    }
}

void Matrix::Subtract(const Matrix &A, const Matrix &B)
{

    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            Set(i, j, A.Get(i, j) - B.Get(i, j));
        }
    }
}

void Matrix::Multiply(const Matrix &A, const Matrix &B)
{
    try
    {
        if (A.GetNColumns() == B.GetNRows())
        {

            for (int i = 0; i < nrows; i++)
            {
                for (int j = 0; j < ncolumns; j++)
                {
                    double n = 0;
                    for (int k = 0; k < A.GetNColumns(); k++)
                    {
                        n += A.Get(i, k) * B.Get(k, j);
                    }
                    Set(i, j, n);
                }
            }
        }
        else
        {
            throw "Matrices do not have the compatible Dimensions";
        }
    }

    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
    }
}

void Matrix::MultiplyScalar(const Matrix &A, double scale)
{

    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            Set(i, j, A.Get(i, j) * scale);
        }
    }
}

// Only valid for vectors represented as 3x1 matrices
void Matrix::CrossProduct(const Matrix &A, const Matrix &B)
{

    Set(0, 0, A.Get(1, 0) * B.Get(2, 0) - A.Get(2, 0) * B.Get(1, 0));
    Set(1, 0, A.Get(2, 0) * B.Get(0, 0) - A.Get(0, 0) * B.Get(2, 0));
    Set(2, 0, A.Get(0, 0) * B.Get(1, 0) - A.Get(1, 0) * B.Get(0, 0));
}

void Matrix::Fill(double value)
{
    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncolumns; j++)
        {
            Set(i, j, value);
        }
    }
}

void Matrix::CopyInto(int row_start, int column_start,
                      const Matrix &A)
{

    for (int i = 0; i < A.GetNRows(); i++)
    {
        for (int j = 0; j < A.GetNColumns(); j++)
        {
            Set(i + row_start, j + column_start, A.Get(i, j));
        }
    }
}

void Matrix::Identity()
{
    Fill(0);

    for (int i = 0; i < nrows; i++)
    {
        Set(i, i, 1);
    }
}

// Only valid for quaternions represented as 4x1 matrices
void Matrix::QuaternionNormalise(const Matrix &q)
{
    MultiplyScalar(q, 1.0 / VectorNorm(q));
}

// Only valid for quaternions represented as 4x1 matrices
void Matrix::RotationMatrixFromQuaternion(const Matrix &q)
{

    NewStackMatrixMacro(q_normed, 4, 1);

    q_normed.QuaternionNormalise(q);

    double qx = q_normed.Get(0, 0);
    double qy = q_normed.Get(1, 0);
    double qz = q_normed.Get(2, 0);
    double qw = q_normed.Get(3, 0);

    Set(0, 0, 1 - 2 * qy * qy - 2 * qz * qz);
    Set(0, 1, 2 * qx * qy - 2 * qz * qw);
    Set(0, 2, 2 * qx * qz + 2 * qy * qw);
    Set(1, 0, 2 * qx * qy + 2 * qz * qw);
    Set(1, 1, 1 - 2 * qx * qx - 2 * qz * qz);
    Set(1, 2, 2 * qy * qz - 2 * qx * qw);
    Set(2, 0, 2 * qx * qz - 2 * qy * qw);
    Set(2, 1, 2 * qy * qz + 2 * qx * qw);
    Set(2, 2, 1 - 2 * qx * qx - 2 * qy * qy);
}

// Only valid for vectors represented as 3x1 matrices
void Matrix::SkewSymmetricFill(const Matrix &A)
{

    Set(0, 0, 0);
    Set(0, 1, -A.Get(2, 0));
    Set(0, 2, A.Get(1, 0));
    Set(1, 0, A.Get(2, 0));
    Set(1, 1, 0);
    Set(1, 2, -A.Get(0, 0));
    Set(2, 0, -A.Get(1, 0));
    Set(2, 1, A.Get(0, 0));
    Set(2, 2, 0);
}

void Matrix::ConcatenateHorizontally(const Matrix &A, const Matrix &B)
{

    for (int i = 0; i < A.GetNRows(); i++)
    {
        for (int j = 0; j < A.GetNColumns(); j++)
        {
            Set(i, j, A.Get(i, j));
        }
    }
    for (int i = 0; i < B.GetNRows(); i++)
    {
        for (int j = 0; j < B.GetNColumns(); j++)
        {
            Set(i, A.GetNColumns() + j, B.Get(i, j));
        }
    }
}

void Matrix::ConcatenateVertically(const Matrix &A, const Matrix &B)
{

    for (int i = 0; i < A.GetNRows(); i++)
    {
        for (int j = 0; j < A.GetNColumns(); j++)
        {
            Set(i, j, A.Get(i, j));
        }
    }
    for (int i = 0; i < B.GetNRows(); i++)
    {
        for (int j = 0; j < B.GetNColumns(); j++)
        {
            Set(A.GetNRows() + i, j, B.Get(i, j));
        }
    }
}

void Matrix::AddRows(int row_to, int row_from, double scale)
{

    for (int i = 0; i < ncolumns; i++)
    {
        Set(row_to, i, Get(row_to, i) + (Get(row_from, i) * scale));
    }
}

void Matrix::MultiplyRow(int row, double scale)
{

    for (int i = 0; i < ncolumns; i++)
    {
        Set(row, i, Get(row, i) * scale);
    }
}

void Matrix::SwitchRows(int row_a, int row_b)
{

    for (int i = 0; i < ncolumns; i++)
    {
        double temp = Get(row_a, i);
        Set(row_a, i, Get(row_b, i));
        Set(row_b, i, temp);
    }
}

void Matrix::RowReduce()
{
    int square;

    if (nrows < ncolumns)
    {
        square = nrows;
    }
    else
    {
        square = ncolumns;
    }

    for (int i = 0; i < square; i++)
    {
        int max_row = i;
        double max_element = fabs(Get(i, i));
        for (int j = i + 1; j < nrows; j++)
        {
            if (fabs(Get(j, i)) > max_element)
            {
                max_element = fabs(Get(j, i));
                max_row = j;
            }
        }

        SwitchRows(i, max_row);

        MultiplyRow(i, 1 / Get(i, i));

        for (int j = 0; j < nrows; j++)
        {
            if (i != j)
            {
                AddRows(j, i, -(Get(j, i)));
            }
        }
    }
}
void Matrix::QuaternionConjugate()
{
    Set(1, 0, -1 * Get(1, 0));
    Set(2, 0, -1 * Get(2, 0));
    Set(3, 0, -1 * Get(3, 0));
    return;
}

void Matrix::QuaternionProductCross(Matrix &a, Matrix &b)
{
    double x = a.Get(0, 0) * b.Get(0, 0) - a.Get(1, 0) * b.Get(1, 0) -
               a.Get(2, 0) * b.Get(2, 0) - a.Get(3, 0) * b.Get(3, 0);
    double y = a.Get(0, 0) * b.Get(1, 0) + a.Get(1, 0) * b.Get(0, 0) -
               a.Get(2, 0) * b.Get(3, 0) + a.Get(3, 0) * b.Get(2, 0);
    double z = a.Get(0, 0) * b.Get(2, 0) + a.Get(2, 0) * b.Get(0, 0) +
               a.Get(1, 0) * b.Get(3, 0) - a.Get(3, 0) * b.Get(1, 0);
    double w = a.Get(0, 0) * b.Get(3, 0) - a.Get(1, 0) * b.Get(2, 0) +
               a.Get(2, 0) * b.Get(1, 0) + a.Get(3, 0) * b.Get(0, 0);
    Set(0, 0, x);
    Set(1, 0, y);
    Set(2, 0, z);
    Set(3, 0, w);
}
void Matrix::QuaternionProductDot(Matrix &a, Matrix &b)
{
    QuaternionProductCross(b, a);
}
double Matrix::DotProduct(const Matrix &A, const Matrix &B)
{
    double dot = 0;

    for (int i = 0; i < 3; i++)
    {
        dot += A.Get(i, 0) * B.Get(i, 0);
    }

    return dot;
}
void Matrix::PrintMatrix()
{

    std::string msg = "\n";
    for (int i = 0; i < nrows; i++)
    {
        msg += "[";
        for (int j = 0; j < ncolumns; j++)
        {
            msg += std::to_string(Get(i, j)) + " ";
        }
        msg += "]\n";
    }

    ROS_INFO_STREAM("Msg:" << msg);
}

void Matrix::CopyMatrix(Matrix &A)
{
    try
    {
        if (ncolumns == A.GetNColumns() && nrows == A.GetNRows())
        {
            for (int i = 0; i < nrows; i++)
            {
                for (int j = 0; j < ncolumns; j++)
                {
                    Set(i, j, A.Get(i, j));
                }
            }
        }
        else
        {
            throw "Matrix::Multiply: Matrices must have the same dimension";
        }
    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
    }
    return;
}
void Matrix::GrammSchmidtNorm(Matrix &A)
{
    try
    {
        int n_rows = A.GetNColumns();
        int n_cols = A.GetNRows();
        Matrix v(n_rows, 1);
        Matrix u(n_rows, 1);     // for each column
        Matrix u_tmp(n_rows, 1); // for each column
        Matrix result(n_rows, n_cols);

        if (ncolumns == A.GetNColumns() && nrows == A.GetNRows() && ncolumns == nrows)
        {

            for (int i = 0; i < A.GetNColumns(); i++)
            {
                for (int j = 0; j < A.GetNRows(); j++)
                {
                    u.Set(j, 0, A.Get(j, i));
                    v.Set(j, 0, A.Get(j, i));
                }
                Matrix proj(n_rows, 1);

                for (int k = i; k > 0; k--)
                {
                    for (int j = 0; j < A.GetNRows(); j++)
                    {
                        u_tmp.Set(j, 0, result.Get(j, i-k));
                    }
                    proj.projection(u_tmp, v);
                    u.Subtract(u, proj);
                }
                for(int j = 0; j<n_rows; j++){
                    result.Set(j,i,u.Get(j,0));
                }
            }
        CopyMatrix(result);
        }
        else
        {
            throw "Matrix :: GrammSchmidtNorm:: Matrices must have the same dimension and be square.";
        }
    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
    }
    return;
}

void Matrix::projection(Matrix &u, Matrix &v)
{
    Matrix proj(u.GetNRows(), 1);
    try
    {
        if (
            u.GetNColumns() == 1 
            && v.GetNColumns() == 1
            && u.GetNRows() == v.GetNRows()
            )
        {
            double sumNum = u.DotProduct(v, u);
            double sumDen = u.DotProduct(u, u);

            double scalar = sumNum / sumDen;
            MultiplyScalar(u, scalar);
        }
        else
        {
            throw "Matrix::projection dimensions of arguments to are incorrect.";
        }
    }
    catch (const char *msg)
    {
        ROS_ERROR_STREAM(msg);
    }
}
void Matrix::ElementSqrt()
{
	/*
	 * Takes the square root of every element in the array
	 *  Not to be confused with the matrix sqaure root.
	 * */
	int rows = GetNRows();
	int cols = GetNColumns();
	int i, j;
        double val2;// holder for retrieved value	
	try{

	for (i = 0; i <rows; i++){
		for (j = 0; j<cols; j++){
			val2 = Get(i,j);
			if (val2>0.0) Set(i,j, sqrt(val2));
			else if (fabs(val2)<0.1)
			{
				Set(i,j,sqrt(fabs(val2)));
			
			}
			else
			{
			throw "Non real result on elementwise matrix sqrt";
			}
		}
	}
	}
	catch (const char *msg)
	{
		ROS_ERROR_STREAM(msg);
	}

	return; 
}

