
Map_Size = size(Map);

Path = [current_L];
Closed = [];

x_g = goal_L(1);
y_g = goal_L(2);

while (1)
    CN = Path(end, :);
    x = CN(1);
    y = CN(2);
    
    
    D = [x, y-1];
    DL = [x-1, y-1];
    L = [x-1, y];
    UL = [(x-1), (y+1)];
    U = [x, y+1];
    UR = [x+1, y+1];
    R = [x+1, y];
    DR = [x+1, y-1];

    %SN = [D; DL; L; UL; U; UR; R; DR];
    SN = [D; L; U; R];
    SN_Size = size(SN);
    
    ESN = [];
    
    %FIND NODE FROM SN THAT HAS CHEAPEST COST AND EXISTS
    for i = 1:SN_Size(1)
        Node = SN(i,:);
        % Ensure that current point is within the bounds of the map
        if ( ((0 < Node(1)) && (Node(1) < Map_Size(1)+1)) && ((0 < Node(2)) && (Node(2) < Map_Size(2)+1)) )
            %Find the current location within the map
            k = Map(Node(1), Node(2));
            %Ensure that point in map is not equal to 0 (obstacle)
            if (k == 1)
                
                %Ensure that current node is not any of the current path
                %points
                Path_Size = size(Path);
                Failed = 0;
                for i = 1:Path_Size(1)
                    P = Path(i, :);
                    a = Node(1);
                    b = Node(2);
                    if ((a == P(1)) && (b == P(2)))
                        Failed = 1;
                    end
                end
                
                %Make sure Node is not closed
                Closed_size = size(Closed);
                if (Failed == 0)
                    if (Closed_size > 0)
                        for i = 1:Closed_size(1)
                            P = Closed(i, :);
                            a = Node(1);
                            b = Node(2);
                            if ((a == P(1)) && (b == P(2)))
                                Failed = 1;
                            end
                        end
                    end
                end
                
                if (Failed == 0)
                    ESN = [ESN; Node];
                end
                
            end
        end
    end
    
    ESN_size = size(ESN);
    if ((ESN_size(1) == 0) && (ESN_size(2) == 0) )
        Closed = [Closed; Path(end, :)];
        Path(end, :) = [];
    else
        LC = CalcBestOption(goal_L, ESN);
        Path = [Path; LC];
    end
    
    CN = Path(end, :);
    if ((CN(1) == goal_L(1)) && ((CN(2) == goal_L(2))))
        break
    end
    
end


