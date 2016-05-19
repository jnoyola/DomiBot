def next_piece(A,end_train):
    n = np.size(A,1);
    init_choice = A[:,end_train-1] + np.transpose(A[end_train-1,:]);
    fix = np.zeros((n,1));
    fix[end_train - 1] = 1;
    init_choice -= fix;
    m = max(init_choice);
    poss_ind = np.where(init_choice);
    poss_pick_up = (np.sum(np.transpose(A),1)) + np.sum(A,1) - np.asmatrix(np.diag(A)).transpose();
    pick_up = np.argmax(poss_pick_up)+1;
    return pick_up