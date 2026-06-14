#ifndef H_gsl_matrix_get_row_m
#define H_gsl_matrix_get_row_m

#include <Eigen/Dense>

int gsl_matrix_get_row_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int row);

int gsl_matrix_get_column_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int column);

int gsl_matrix_get_part_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int s_row, int e_row, int s_column, int e_column);

int gsl_matrix_get_part2_m(Eigen::MatrixXd & matrix_out, const Eigen::MatrixXd & matrix_in, int s_row, int e_row, int s_column, int e_column);

#endif
