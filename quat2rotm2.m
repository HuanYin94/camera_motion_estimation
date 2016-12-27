function M = quat2rotm2(q)

assert(isvector(q) && length(q) == 4);

q = q / norm(q);

M = eye(3) - 2 * q(4) * skew(q(1:3)) + 2 * skew(q(1:3)) ^ 2;