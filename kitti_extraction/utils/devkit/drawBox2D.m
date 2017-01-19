function drawBox2D(h,box, occlusion, object_type)

% set styles for occlusion and truncation
occ_col    = {'w', 'g','y','r'};

% show rectangular bounding boxes
pos = [box.x1,box.y1,box.x2-box.x1+1,box.y2-box.y1+1];
rectangle('Position',pos,'EdgeColor',occ_col{occlusion+2},...
          'LineWidth',6,'parent',h(1).axes)
rectangle('Position',pos,'EdgeColor','k','LineWidth',2,'parent', h(1).axes)

% plot label
label_text = sprintf('%s',object_type);
x = (box.x1+box.x2)/2;
y = box.y1;
text(x,max(y-5,40),label_text,'color',occ_col{occlusion+2},...
     'BackgroundColor','k','HorizontalAlignment','center',...
     'VerticalAlignment','bottom','FontWeight','bold',...
     'FontSize',16,'parent',h(1).axes);
   