function z_o = new1_o_p(a,b,o)
if isempty(o)
   z_o=0 ;
else
point=[a b];
for s=2:size(o)
    point=[point;a b];
end

%z_o=sum(1.4.^(1./((sum((o-point).^2,2).^(1/2).*0.4)))-1);
%f=(1.4.^(1./((X.^2)+(Y.^2))^(1/2).*0.4)-1);
z_o = sum(1.4 .^ (1 ./ (sqrt(sum((o - point) .^ 2, 2)) * 0.4)) - 1);

end
end

