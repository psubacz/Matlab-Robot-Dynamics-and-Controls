function add_blk(newblock,baseblock,refPos,offset,size)
% pos = get_param(refblock,'Position');
% orient = get_param(refblock,'Orientation');

% pos
% pos = [posReff(1) posReff(2)-siz(2)/2 posReff(1)+siz(1) posReff(2)+siz(2)/2]
pos = [refPos(1)+offset(1) refPos(2)+offset(2)-size(2)/2 refPos(3)+offset(1)+size(1) refPos(4)+offset(2)+size(2)/2];

% add_block(newblock,baseblock,'Position',pos,'Orientation',orient);
add_block(newblock,baseblock,'Position',pos);
end

