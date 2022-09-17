use nalgebra::{Dim, DMatrix, Matrix, RawStorage, Scalar};
use num_traits::Zero;

pub fn diagonal_block_matrix<T: Scalar + Zero>(matrices: &[DMatrix<T>]) -> DMatrix<T> {
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
            c..next_c,
        )).copy_from(&matrix);
        r = next_r;
        c = next_c;
    }
    block_diagonal_matrix
}

pub fn into_dynamic<T, R, C, S>(matrix: Matrix<T, R, C, S>) -> DMatrix<T> where
    T: Scalar + Zero, R: Dim, C: Dim, S: RawStorage<T, R, C> {
    DMatrix::from_fn(matrix.nrows(), matrix.ncols(), |r, c| matrix[(r, c)].clone())
}

pub fn tile<T, R, C, S>(matrix: Matrix<T, R, C, S>, v: usize, h: usize) -> DMatrix<T> where
    T: Scalar + Zero, R: Dim, C: Dim, S: RawStorage<T, R, C> {
    let nr = matrix.nrows();
    let nc = matrix.ncols();
    DMatrix::from_fn(nr * v, nc * h, |r, c| {
        matrix[(r % nr, c % nc)].clone()
    })
}

pub fn horizontal_stack<T, R, C, S>(matrices: &[Matrix<T, R, C, S>]) -> DMatrix<T> where
    T: Scalar + Zero, R: Dim, C: Dim, S: RawStorage<T, R, C> {
    assert!(matrices.len() > 0);
    let nr = matrices[0].nrows();
    let nc = matrices[0].ncols();
    DMatrix::from_fn(nr, nc * matrices.len(), |r, c| {
        matrices[c / nc][(r, c % nc)].clone()
    })
}

pub fn vertical_stack<T, R, C, S>(matrices: &[Matrix<T, R, C, S>]) -> DMatrix<T> where
    T: Scalar + Zero, R: Dim, C: Dim, S: RawStorage<T, R, C> {
    assert!(matrices.len() > 0);
    let nr = matrices[0].nrows();
    let nc = matrices[0].ncols();
    DMatrix::from_fn(nr * matrices.len(), nc, |r, c| {
        matrices[r / nr][(r % nr, c)].clone()
    })
}
