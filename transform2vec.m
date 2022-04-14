function X = transform2vec(A,dim)
switch length(A)
    case 3
        translation = A(1:2,3);
        rx =A(1:2,1);
        u = rx / norm(rx);
        rotation    = atan2(u(2),u(1));
        
    case 4
        translation = A(1:3,4);
        R = A(1:3,1:3);
        
        ax  = atan2(R(3,2),R(3,3));
        ay  = atan2(-R(3,1), sqrt( R(3,2)^2 + R(3,3)^2 ) );
        az  = atan2(R(2,1),R(1,1));
        
        if exist('dim','var')
            if dim == 6
                rotation =[ax; ay; az];
            elseif dim == 4
                rotation = az;
            end
        else
            rotation = az;
        end
        
        
    otherwise
        error('wrong size for Transform')
end


X = [translation; rotation];
