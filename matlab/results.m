function [measures, time] = results(filename, topic)
clear measures
clear time

bag = rosbag(filename);
bagselet1 = select(bag,'Topic', topic);

msg = readMessages(bagselet1, "DataFormat",'struct');

n = bagselet1.NumMessages;
sec = msg{1}.Header.Stamp.Sec;
nsec = msg{1}.Header.Stamp.Nsec;
    
    X=double.empty(n,0);
    Y=double.empty(n,0);
    Z=double.empty(n,0);
    x=double.empty(n,0);
    y=double.empty(n,0);
    z=double.empty(n,0);
    w=double.empty(n,0);
    time=double.empty(n,0);


    for i=1:n
        if not(isempty(msg{i}.Transforms))
            X(i)=msg{i}.Transforms.Transform.Translation.X;
            Y(i)=msg{i}.Transforms.Transform.Translation.Y;
            Z(i)=msg{i}.Transforms.Transform.Translation.Z;
            x(i)=msg{i}.Transforms.Transform.Rotation.X;
            y(i)=msg{i}.Transforms.Transform.Rotation.Y;
            z(i)=msg{i}.Transforms.Transform.Rotation.Z;
            w(i)=msg{i}.Transforms.Transform.Rotation.W;
        else
            X(i)=X(i-1);
            Y(i)=Y(i-1);
            Z(i)=Z(i-1);
            x(i)=x(i-1);
            y(i)=y(i-1);
            z(i)=z(i-1);
            w(i)=w(i-1);
        end
    end
     
    
    for i=1:n
        time(i)=double(msg{i}.Header.Stamp.Sec-sec)+(double(msg{i}.Header.Stamp.Nsec)-double(nsec))*1e-9;
    
    end
   
    
    measures = [X; Y; Z; x; y; z; w];
    
end

    