#ifndef DATA_ASSOCIATION_H
#define DATA_ASSOCIATION_H


Eigen::MatrixXf genF(int idx)
{
    Eigen::MatrixXf F; 
    int size = STATE_SIZE + LM_SIZE; 
    F = Eigen::MatrixXf::Zero(N_t, size);
    F(0,0) = 1;
    F(1,1) = 1;
    F(2,2) = 1;
    F(3,3) = 1;
    F(4,4) = 1;
    F(5+2*idx, 5+2*idx) = 1;
    F(6+2*idx, 6+2*idx) = 1;
    return F;
}
#endif