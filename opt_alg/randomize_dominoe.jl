function randomize_dominoes(n,count)

    m = n^2/2 + n/2
    candidate = 1:m;
    tiles = zeros(count,1);

    partition = zeros(n,1);
    for i = 1:n
        partition[i] = n + 1 - i ;
    end
    A = zeros(n,n);

    c_partition = cumsum(partition);
    c_partition = [0 ; c_partition];

    for i = 1:count
        x = rand(1:m);
        while length(intersect(x,tiles)) > 0
            x = rand(1:m);
        end
        tiles[i] = x;

        r = findfirst((c_partition - x + 1 ) .> 0);
        row = r - 1;
        col = round(Int,x - c_partition[r-1]) + row - 1;
        A[row,col] = 1;
    end 

        return A;
end