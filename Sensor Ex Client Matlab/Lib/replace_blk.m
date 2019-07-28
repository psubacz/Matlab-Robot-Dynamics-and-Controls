function replace_blk(oldblock,newblock)
pos = get_param(oldblock,'Position');
orient = get_param(oldblock,'Orientation');

delete_block(oldblock);

add_blk(newblock,oldblock,0);
% add_block(newblock,oldblock,'Position',pos,'Orientation',orient);
end

