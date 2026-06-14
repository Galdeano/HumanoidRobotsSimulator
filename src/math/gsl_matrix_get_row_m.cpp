#include <Eigen/Dense>
#include "gsl_matrix_get_row_m.h"

int gsl_matrix_get_row_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int row)
{
    matrix_out = matrix_in.row(row);
    return 0;
}

int gsl_matrix_get_column_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int column)
{
    matrix_out = matrix_in.col(column);
    return 0;
}

int gsl_matrix_get_part_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int s_row, int e_row, int s_column, int e_column)
{
    matrix_out = matrix_in.block(s_row, s_column, e_row - s_row, e_column - s_column);
    return 0;
}

int gsl_matrix_get_part2_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int s_row, int e_row, int s_column, int e_column)
{
    matrix_out = matrix_in.block(s_row, s_column, e_row - s_row, e_column - s_column);
    return 0;
}
