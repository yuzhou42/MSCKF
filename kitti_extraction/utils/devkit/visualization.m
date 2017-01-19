function varargout = visualization(mode,image_dir,varargin)

switch mode
  
  % init figure
  case 'init'
    
    % create figure using size of first image in repository
    fig = figure(1);
    img = imread(sprintf('%s/%010d.png',image_dir,0));
    set(fig,'position',[100,100,0.8*size(img,2),0.8*2*size(img,1)]);
    h(1).axes = axes('position',[0,0.5,1,0.5]);
    h(2).axes = axes('position',[0,0,1,0.5]);
    varargout{1} = h;
    
  % update figure
  case 'update'
    
    % unpack input arguments
    h        = varargin{1};
    img_idx  = varargin{2};
    nimages  = varargin{3};
    
    % read image
    img = imread(sprintf('%s/%010d.png',image_dir,img_idx));
    
    % clear axes, draw image
    cla(h(1).axes); cla(h(2).axes);
    imshow(img,'parent',h(1).axes); axis(h(1).axes,'image','off'); hold(h(1).axes, 'on');
    imshow(img,'parent',h(2).axes); axis(h(2).axes,'image','off'); hold(h(2).axes, 'on');
    
    % title
    text(size(img,2)/2,3,sprintf('2D Bounding Boxes'),'parent',h(1).axes,'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(size(img,2)/2,3,sprintf('3D Bounding Boxes'),'parent',h(2).axes,'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    
    % legend
    text(0,00,'Not occluded','parent',h(1).axes,'color','g','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,30,'Partly occluded','parent',h(1).axes,'color','y','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,60,'Fully occluded','parent',h(1).axes,'color','r','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,90,'Unknown','parent',h(1).axes,'color','w','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    
    % frame number
    text(size(img,2),0,sprintf('frame %d/%d',img_idx,nimages-1), 'parent', h(1).axes,'color','g','HorizontalAlignment','right','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black', 'Interpreter','none');
    
    % usage instructions
    text(size(img,2)/2,size(img,1),sprintf('''SPACE'': Next Image  |  ''-'': Previous Image  |  ''x'': +100  |  ''y'': -100 | ''q'': quit'), 'parent', h(2).axes,'color','g','HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',14,'FontWeight','bold', 'BackgroundColor','black');
end
