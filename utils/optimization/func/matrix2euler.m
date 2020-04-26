function euler = matrix2euler(matrix)
    euler_angle = rotm2eul(matrix(1:3, 1:3));
    euler_vec = matrix(1:3, 4);
    euler = [euler_angle(1) euler_angle(2) euler_angle(3) euler_vec(1) euler_vec(2) euler_vec(3)]';
end