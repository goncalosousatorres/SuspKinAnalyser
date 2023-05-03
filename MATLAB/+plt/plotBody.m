function plotBody(body,lineformat,color,close)

if (nargin < 4 || close==1) && size(body,2) > 2
    body(:,end+1) = body(:,1);
end
plot3(body(1,:), body(2,:), body(3,:),strcat(lineformat,color));

end
