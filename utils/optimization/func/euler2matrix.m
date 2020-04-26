function eulerMatrix = euler2matrix(euler)
    eulerMatrix = eye(4);
    eulerMatrix(1:3, 1:3) = eul2rotm([euler(1) euler(2) euler(3)]);
    eulerMatrix(1:3, 4) = [euler(4) euler(5) euler(6)]';
end