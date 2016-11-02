function BestOption = CalcBestOption(goal_L, Options)

        x_g = goal_L(1);
        y_g = goal_L(2);
        
        Size = size(Options);
        TC = [];
        
        for i = 1:Size(1)
            Current_Node = Options(i, :);
            x_n = Current_Node(1);
            y_n = Current_Node(2);

            %Determine total cost of current node
            f = sqrt((x_g-x_n)^2 + (y_g-y_n)^2);
            
            %Append to total cost vector
            TC(i) = f;
        end
        
        %Find minimum value
        M = min(TC);
        I = find(TC == M, 1, 'last');
        
        %Find corresponding node and append to Path array.
        BestOption = Options(I, :);
end
