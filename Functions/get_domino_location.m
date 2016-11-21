function Location = get_domino_location(DominoValue)
%get_domino_location returns the goal state of the specified domino

Location = [850, 385];

if( (DominoValue(1) == 0) && (DominoValue(2) == 0) )
    Location = [770,65];
elseif( (DominoValue(1) == 0) && (DominoValue(2) == 1) )
    Location = [930,65];
elseif( (DominoValue(1) == 1) && (DominoValue(2) == 1) )
    Location = [1040,65];
elseif( (DominoValue(1) == 1) && (DominoValue(2) == 2) )
    Location = [1150,65];
elseif( (DominoValue(1) == 1) && (DominoValue(2) == 3) )
    Location = [1270,65]; 
elseif( (DominoValue(1) == 1) && (DominoValue(2) == 4) )
    Location = [1400,65];
elseif( (DominoValue(1) == 1) && (DominoValue(2) == 5) )
    Location = [1540,65];
elseif( (DominoValue(1) == 1) && (DominoValue(2) == 6) )
    Location = [1635,90];
elseif( (DominoValue(1) == 0) && (DominoValue(2) == 2) )
    Location = [1600,285];
elseif( (DominoValue(1) == 2) && (DominoValue(2) == 2) )
    Location = [1565,350];
elseif( (DominoValue(1) == 2) && (DominoValue(2) == 3) )
    Location = [1440,450];
elseif( (DominoValue(1) == 2) && (DominoValue(2) == 4) )
    Location = [1320,550];
elseif( (DominoValue(1) == 2) && (DominoValue(2) == 5) )
    Location = [1180,600];
elseif( (DominoValue(1) == 2) && (DominoValue(2) == 6) )
    Location = [1060,650];
elseif( (DominoValue(1) == 0) && (DominoValue(2) == 3) )
    Location = [920,680];
elseif( (DominoValue(1) == 3) && (DominoValue(2) == 3) )
    Location = [780,680];
elseif( (DominoValue(1) == 3) && (DominoValue(2) == 4) )
    Location = [650,650];
elseif( (DominoValue(1) == 3) && (DominoValue(2) == 5) )
    Location = [500,600];
elseif( (DominoValue(1) == 3) && (DominoValue(2) == 6) )
    Location = [340,550];
elseif( (DominoValue(1) == 0) && (DominoValue(2) == 4) )
    Location = [220,450];
elseif( (DominoValue(1) == 4) && (DominoValue(2) == 4) )
    Location = [115,350];
elseif( (DominoValue(1) == 4) && (DominoValue(2) == 5) )
    Location = [80,245];
elseif( (DominoValue(1) == 4) && (DominoValue(2) == 6) )
    Location = [65,90];
elseif( (DominoValue(1) == 0) && (DominoValue(2) == 5) )
    Location = [160,65];
elseif( (DominoValue(1) == 5) && (DominoValue(2) == 5) )
    Location = [280,65];
elseif( (DominoValue(1) == 5) && (DominoValue(2) == 6) )
    Location = [410,65];
elseif( (DominoValue(1) == 0) && (DominoValue(2) == 6) )
    Location = [530,65];
elseif( (DominoValue(1) == 6) && (DominoValue(2) == 6) )
    Location = [660,65];
end

Location = fliplr(Location);

end

