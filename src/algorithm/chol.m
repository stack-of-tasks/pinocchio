//
// Copyright (c) 2015 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

parent = [ 0 1    2 3    2 5    2 7    8 9    8 11 ];
%parent = [ 0 1    2 3    2 5    2 7    8 9 10   8 12 13 ];
%parent = [ 0 1 2 3 4 5   6 7 8 9 10 11    6 13 14 15 16 17   6 19    20 21 ...
%           22 23 24 25       20 27 28 29 30 31 ];
n = size(parent,2);
M = zeros(n);
for i=1:n
    j=i;
    while j>0
        M(i,j) = rand;
        M(j,i) = M(i,j);
        j=parent(j);
    end
end
M
% M = [
%  32  -8  -1   2   2   1 -14   2  -3   0   3  -4   0   1
%  -8  27   2  -1   1   2 -11  12   6   2   1   3   0   1
%  -1   2   4   1   0   0   0   0   0   0   0   0   0   0
%   2  -1   1   3   0   0   0   0   0   0   0   0   0   0
%   2   1   0   0   3   1   0   0   0   0   0   0   0   0
%   1   2   0   0   1   2   0   0   0   0   0   0   0   0
% -14 -11   0   0   0   0  29 -13  -1   0   0   5   4   0
%   2  12   0   0   0   0 -13  19   4   2   2   2  -2   2
%  -3   6   0   0   0   0  -1   4   9   2  -1   0   0   0
%   0   2   0   0   0   0   0   2   2   3   2   0   0   0
%   3   1   0   0   0   0   0   2  -1   2   4   0   0   0
%  -4   3   0   0   0   0   5   2   0   0   0   7   1   1
%   0   0   0   0   0   0   4  -2   0   0   0   1   3   1
%   1   1   0   0   0   0   0   2   0   0   0   1   1   1];

tree = zeros(1,n);
for i=n:-1:2
    if(tree(i)==0) tree(i) = i; end
    tree(parent(i)) = max(tree(parent(i)), tree(i) );
end

method=3

if method==1
    D = zeros(n,1);
    U = eye(n);
    for i=n:-1:1
        for j=n:-1:i+1
            subtree = j+1:tree(j);
            U(i,j) = inv(D(j)) * (M(i,j) - U(i,subtree)*diag(D(subtree))*U(j,subtree)');
        end
        subtree = i+1:tree(i);
        D(i) = M(i,i) - U(i,subtree)*diag(D(subtree))*U(i,subtree)';
    end
end

if method==2
    Msav = M;
    D = zeros(n,1);
    U = eye(n);
    for k=n:-1:1
        i=parent(k);
        while i>0
            a = M(i,k) / M(k,k);
            %ak = [i k a]
            M(i,i) = M(i,i) - a * M(i,k);
            j = parent(i);
            while j>0
                M(j,i) = M(j,i) - a * M(j,k);
                %ij= [ i j M(i,j)]
                j=parent(j);
            end
            M(i,k) = a;
            %ik = [ i k M(i,k)]
            i=parent(i);
        end
    end
    
    U = triu(M,1) + eye(n); 
    D = diag(M);
    M = Msav;
end


% i<k  =>  U(i,k) != 0,  U(k,k) = 1
% M(i,k) = sum_{m=1}^{n} U(i,m) D(m) U(k,m)'
%        = sum_{m=k+1}^{n} U(i,m) D(m) U(k,m)' + U(i,k) D(k)
% =>   U(i,k) = ( M(i,k) - sum_{m>k} ... ) D{k}^{-1}

if method==3
    D = zeros(n,1);
    U = eye(n);
    for k=n:-1:1
        subtree = k+1:tree(k);
        D(k) = M(k,k) - U(k,subtree)*diag(D(subtree))*U(k,subtree)';
        i=parent(k);
        while i>0
            U(i,k) = (M(i,k) - U(i,subtree)*diag(D(subtree))*U(k,subtree)') ...
                     / D(k);
            i=parent(i);
        end
    end
end


norm(U*diag(D)*U' - M)