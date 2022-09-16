use nalgebra::{Dim, DimName, DMatrix, Matrix, RawStorage, Scalar};
use num_traits::Zero;

pub fn diagonal_block_matrix<T: Scalar + Zero>(matrices: Vec<DMatrix<T>>) -> DMatrix<T> {
    let total_rows = matrices.iter().map(DMatrix::nrows).sum();
    let total_cols = matrices.iter().map(DMatrix::ncols).sum();
    let mut block_diagonal_matrix = DMatrix::zeros(total_rows, total_cols);
    // TODO: use ranges instead of two diff variables?
    let mut r = 0;
    let mut c = 0;
    for matrix in matrices.into_iter() {
        let next_r = r + matrix.nrows();
        let next_c = c + matrix.ncols();
        block_diagonal_matrix.index_mut((
            r..next_r,
            c..next_c
        )).copy_from(&matrix);
        r = next_r;
        c = next_c;
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

pub fn tile<T, R, C, S>(matrix: Matrix<T, R, C, S>, r: usize, c: usize) -> DMatrix<T> where
    T: Scalar + Zero,
    R: Dim + DimName, C: Dim + DimName,
    S: RawStorage<T, R, C> {
    let nr = matrix.nrows();
    let nc = matrix.ncols();
    let mut tiled_matrix: DMatrix<T> = DMatrix::zeros(nr * r, nc * c);
    for ir in 0..r {
        for ic in 0..c {
            tiled_matrix.index_mut((
                ir * nr..(ir + 1) * nr,
                ic * nc..(ic + 1) * nc,
            )).copy_from(&matrix);
        }
    }
    tiled_matrix
}

pub fn horizontal_stack<T, R, C, S>(matrices: Vec<Matrix<T, R, C, S>>) -> DMatrix<T> where
    T: Scalar + Zero,
    R: Dim + DimName, C: Dim + DimName,
    S: RawStorage<T, R, C> {
    let max_rows = matrices.iter().map(|m| m.nrows()).max().expect("No max");
    let total_cols = matrices.iter().map(|m| m.ncols()).sum();
    let mut horizontal_matrix: DMatrix<T> = DMatrix::zeros(max_rows, total_cols);
    let mut c = 0;
    for matrix in matrices {
        let next_c = c + matrix.ncols();
        horizontal_matrix.index_mut((.., c..next_c)).copy_from(&matrix);
        c = next_c;
    }
    horizontal_matrix
}
