function z_o = o_p(a,b,o)
if isempty(o)
   z_o=0 ;
else
point=[a b];
for s=2:size(o)
    point=[point;a b];
end

z_o=sum((sum(((point-o).^2),2).^(1/2)).^(-1));
end
end

