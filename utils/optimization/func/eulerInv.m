function euler_inv = eulerInv(euler)
    matrix = euler2matrix(euler);
    matrixInv = inv(matrix);
    euler_inv = matrix2euler(matrixInv);
end