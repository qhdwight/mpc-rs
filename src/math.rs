use na::{Dim, DimName, DMatrix, DVector, Matrix, RawStorage, Scalar};
use num_traits::Zero;

pub fn diagonal_block_matrix<T: Scalar + Zero>(matrices: DVector<DMatrix<T>>) -> DMatrix<T> {
    let total_rows = matrices.iter().map(DMatrix::nrows).sum();
    let total_cols = matrices.iter().map(DMatrix::nrows).sum();
    let mut block_diagonal_matrix = DMatrix::zeros(total_rows, total_cols);
    // TODO: use ranges instead of two diff variables?
    let mut r = 0;
    let mut c = 0;
    for matrix in matrices.into_iter() {
        let nr = r + matrix.nrows();
        let nc = c + matrix.ncols();
        block_diagonal_matrix.index_mut((r..nr, c..nc)).copy_from(&matrix);
        r = nr;
        c = nc;
    }
    block_diagonal_matrix
}

pub fn into_dynamic<T, R, C, S>(matrix: Matrix<T, R, C, S>) -> DMatrix<T> where
    T: Scalar + Zero,
    R: Dim + DimName, C: Dim + DimName,
    S: RawStorage<T, R, C> {
    let mut dynamic_matrix = DMatrix::zeros(matrix.nrows(), matrix.ncols());
    dynamic_matrix.copy_from(&matrix);
    dynamic_matrix
}

pub fn tile<T, R, C, S>(matrix: Matrix<T, R, C, S>, count: usize) -> DMatrix<T> where
    T: Scalar + Zero,
    R: Dim + DimName, C: Dim + DimName,
    S: RawStorage<T, R, C> {
    let mut tiled_matrix = DMatrix::zeros(matrix.nrows(), matrix.ncols() * count);
    for _ in 0..count {
        let i = count * C;
        tiled_matrix.index_mut(()).copy_from(&matrix);
    }
    tiled_matrix
}