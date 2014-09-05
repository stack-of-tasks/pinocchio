clear U L M parent tree 

parent = [ 0 1    2 3    2 5    2 7    8 9    8 11 ];
n=12;
M = zeros(n);
for i=1:n
    j=i;
    while j>0
        M(i,j) = rand;
        M(j,i) = M(i,j);
        L(i,j) = 1;
        U(j,i) = 1;
        j=parent(j);
    end
end
    
tree = zeros(1,n);
for i=n:-1:2
    if(tree(i)==0) tree(i) = i; end
    tree(parent(i)) = max(tree(parent(i)), tree(i) );
end

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

Msav = M;
D = zeros(n,1);
U = eye(n);
for k=n:-1:1
    i=parent(k);
    while i>0
        a = M(i,k) / M(k,k);
        M(i,i) = M(i,i) - a * M(i,k);
        j = parent(i);
        while j>0
            M(j,i) = M(j,i) - a * M(j,k);
            j=parent(j);
        end
        M(i,k) = a;
        i=parent(i);
    end
end

U = triu(M,1) + eye(n); 
D = diag(M);
M = Msav;
norm(U*diag(D)*U' - M)