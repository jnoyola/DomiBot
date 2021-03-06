import numpy as np

def desired_targets(cv_data,cv_ends,n,offset):

    # constructing A and sorting through dominos

    A = np.zeros((n,n))   # size of A matrix
    num_dominoes = len(cv_data)   # number of dominoes
    dom = list()   # extracting CV domino data
    dom_sort = list()   # sort them so the smaller number is the first column
    dom_perm = list()   # permutation: if there was a swapping or not

    for i in range(0,num_dominoes):          # sorting out the dominoes and constructing A
        dom.append([cv_data[i][0][0],cv_data[i][1][0]])
        dom_sort.append(sorted(dom[i]));
        dom_perm.append(sorted(range(len(dom[i])), key=lambda k: dom[i][k]))
        A[dom_sort[i][0],dom_sort[i][1]] = 1;

    A = A.astype(int)
    
    
    # choosing a piece to play
    
    end_train1 = cv_ends[0][0];   # renaming the train ends
    end_train2 = cv_ends[1][0];
    n = np.size(A,1);   # determine size of A
    AA = np.transpose(A)  # need a transpose for calculations
    poss_dom_to_pick_up = list()  # possible domino to pick for each end

    init_choice = A[:,end_train1] + np.transpose(A[end_train1,:]);  # Find playable pieces for end 1

    if np.sum(init_choice) == 0:  # if no playable piece make is available, it is not an option
        V1 = 0;
        poss_dom_to_pick_up.append(0)
    else :
        init_choice[end_train1] -= A[end_train1,end_train1];  # undo double counting of diagonals
        poss_ind = list()  # index of possible playable list for end 1
        poss_pick_up = list() # value of possible playable dominoes for end 1

        for i in range(0,n):
            if init_choice[i] == 1:  # if it is a playable piece then...
                poss_ind.append(i)  # add the index to poss_ind
                poss_pick_up.append(int(sum(list(A[i,:])) + sum(list(A[:,i])) - A[i,i]))  # calculate value of possible piece
        (V1,ind_pick_up) = max((v,i) for i,v in enumerate(poss_pick_up)) # find the maximum value and its index
        poss_dom_to_pick_up.append([end_train1,poss_ind[ind_pick_up]]) # best candidate for end 1


    init_choice = A[:,end_train2] + np.transpose(A[end_train2,:]);  # Find playable pieces for end 2

    if np.sum(init_choice) == 0: # if no playable piece make is available, it is not an option
        V2 = 0;
        poss_dom_to_pick_up.append(0)
    else :
        init_choice[end_train2] -= A[end_train2,end_train2];  # undo double counting of diagonals
        poss_ind = list()  # index of possible playable list for end 1
        poss_pick_up = list()  # value of possible playable dominoes for end 1
        
        for i in range(0,n):
            if init_choice[i] == 1:  # if it is a playable piece then...
                poss_ind.append(i)  # add the index to poss_ind
                poss_pick_up.append(int(sum(list(A[i,:])) + sum(list(A[:,i])) - A[i,i]))  # calculate value of possible piece
        (V2,ind_pick_up) = max((v,i) for i,v in enumerate(poss_pick_up))  # find the maximum value and its index
        poss_dom_to_pick_up.append([end_train2,poss_ind[ind_pick_up]])  # best candidate for end 2

    V_ends = [V1, V2]; # value of end 1 and end 2
    (V,ind_end_to_play) = max((v,i) for i,v in enumerate(V_ends)) # pick end with greater value
    piece_to_play = poss_dom_to_pick_up[ind_end_to_play]; # pick the domino from the possible list with the greater value
    
    
    # convert to right format to match with cv_data_sorted
    to_be_matched = sorted(piece_to_play);
    end_to_play = cv_ends[ind_end_to_play][0];
    
    # finding which index the desired domino is in from the list
    for i in range(0,num_dominoes):
        if to_be_matched == dom_sort[i]:
            ind_desired_dom = i
    # find out if the domino was permuted
    target_dom = dom[ind_desired_dom]
    if target_dom[0] == end_to_play:
        ind_contact_end = 0
        ind_next_play_end = 1
    else:
        ind_contact_end = 1
        ind_next_play_end = 0
    # extracting desired orientation and position
    # pick up
    pickup_desired_orientation = cv_data[ind_desired_dom][ind_next_play_end][2]
    x_desired = float(cv_data[ind_desired_dom][ind_next_play_end][1][0] + cv_data[ind_desired_dom][ind_contact_end][1][0])/2.0
    y_desired = float(cv_data[ind_desired_dom][ind_next_play_end][1][1] + cv_data[ind_desired_dom][ind_contact_end][1][1])/2.0
    pickup_desired_position = [x_desired, y_desired]
    target_pick_up = [pickup_desired_position, pickup_desired_orientation]
    
    # put down
    putdown_desired_orientation = cv_ends[ind_end_to_play][2]
    x_desired = cv_ends[ind_end_to_play][1][0] + offset[0]
    y_desired = cv_ends[ind_end_to_play][1][1] + offset[1]
    putdown_desired_position = [x_desired, y_desired]
    target_put_down = [putdown_desired_position, putdown_desired_orientation]
    
    
    
    return [[piece_to_play[dom_perm[ind_desired_dom][0]],piece_to_play[dom_perm[ind_desired_dom][1]]], target_pick_up, target_put_down]


# Example to test
cv_data = [[(3, (431, 197), 0.51914611424652302), (5, (386, 219), -2.7367008673047097)], [(4, (607, 173), 1.3986055122719576), (0, (595, 223), -1.7942729279355296)], [(0, (283, 178), 0.75837771421018374), (6, (246, 213), -2.4116929954380617)], [(6, (491, 146), 1.4909663410826592), (6, (487, 196), -1.7429871413178355)], [(4, (370, 153), -0.41012734054149091), (3, (324, 133), -3.5363837732895549)], [(2, (288, 90), 1.5707963267948966), (4, (286, 142), -1.6614562139956417)], [(6, (478, 110), -0.4461055489434036), (4, (432, 90), -3.5713549326864817)], [(3, (557, 60), 1.5707963267948966), (2, (553, 113), -1.6575346654708816)], [(2, (395, 88), -0.81396182123620819), (0, (362, 49), -3.9572845769060168)]]
cv_ends = [(0, (431, 197), 0.51914611424652302), (5, (386, 219), -2.7367008673047097)]
n = 7;
offset = [0.05, 0.05]
[desired_piece, target_pick_up, target_put_down] = desired_targets(cv_data,cv_ends,n,offset)
print([desired_piece, target_pick_up, target_put_down])


