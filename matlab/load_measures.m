function [measures, time, error] = load_measures(filename)
    bag = rosbag(filename);
    msg = readMessages(bag, "DataFormat",'struct');
    
    n = bag.NumMessages;
    
    X=double.empty(n,0);
    Y=double.empty(n,0);
    Z=double.empty(n,0);
    x=double.empty(n,0);
    y=double.empty(n,0);
    z=double.empty(n,0);
    w=double.empty(n,0);
    error=double.empty(n,0);
    time=double.empty(n,0);
    
    sec = msg{1}.Header.Stamp.Sec;
    nsec = msg{1}.Header.Stamp.Nsec;
   
        
   
    for i=1:n
        X(i)=msg{i}.Transforms.Transform.Translation.X;
        Y(i)=msg{i}.Transforms.Transform.Translation.Y;
        Z(i)=msg{i}.Transforms.Transform.Translation.Z;
        x(i)=msg{i}.Transforms.Transform.Rotation.X;
        y(i)=msg{i}.Transforms.Transform.Rotation.Y;
        z(i)=msg{i}.Transforms.Transform.Rotation.Z;
    end
    
    for i=1:n
        w(i)=msg{i}.Transforms.Transform.Rotation.W;
    end
     
    for i=1:n
        error(i)=msg{i}.Transforms.ObjectError;
    end
    
    for i=1:n
        time(i)=double(msg{i}.Header.Stamp.Sec-sec)+(double(msg{i}.Header.Stamp.Nsec)-double(nsec))*1e-9;
    
    end
    
    
    measures = [X; Y; Z; x; y; z; w];
      
end



