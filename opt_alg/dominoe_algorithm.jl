
include("randomize_dominoe.jl")


n = 4;
longest = 4;
count = 4
A = randomize_dominoes(n,count);
# println(round(Int,A));

# continue randomising the dominoes until we get feasible set
if sum(diag(A)) == 0.0
    starting_dom = false;
    else starting_dom = true;
end
while starting_dom == false
    A = randomize_dominoes(n);
    if round(Int,sum(diag(A))) > 0 
        starting_dom = true
    end
end

# print out the avaliable dominoes
println(round(Int,A));

# choose a start dominoe which has most common tiles
possible_start = find(diag(A));
V = zeros(n,1);
value = sum(A,1)' + sum(A,2);
V[possible_start] = value[possible_start];
starting_dom = indmax(V)

# initialising dominoe
current_from = starting_dom;
current_to = starting_dom;

# remove used dominoe from playing field
A[current_from, current_to] = 0;
count -= 1;     # number of used tiles
next_possible = n - 1;     
println(current_from,",",current_to)

while (length(next_possible) > 0  )
    V = zeros(n,1);
    # find possible tiles to play
    next_possible = union(find(A[current_to,:]),find(A[:,current_to]));
    println(next_possible)
    # see which number is more common
    value = sum(A,1)' + sum(A,2);
    V[next_possible] = value[next_possible];
    # play the most common allowable tile
    next_piece = indmax(V)
    # play the dominoe and update current piece
    current_from = current_to; 
    current_to = next_piece;
    println(current_from,",",current_to)
    # remove from playble field
    A[current_from, current_to] = 0;
    A[current_to, current_from] = 0;
    count -= 1;
    if count == 0
        return 
    end
    next_possible = union(find(A[current_to,:]),find(A[:,current_to]));
end

println("Length of train: ",longest - count)


