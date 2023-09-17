clear
%%%%%%%%%%%%%%%%%%%%%%%Define Before Run%%%%%%%%%%%%%%%%%%      Test Value
heading = 270 ; %270 deg implied down
                %180 deg toward LZ ?
                %90 deg implied up
                %0 deg opposite LZ ?
command = 'm';  %l localization %m block detect %n block delievery 
LZ_p = 0.6;    %L/Z probability                                Test Value


m_u = [];
m_m = [];
LZ = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Map Setup%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];

ultra = zeros(size(bw));
for sec_row = 1:4:dim2,
    for sec_col = 1:4:dim1,
        segRow = M(sec_row+2, sec_col:sec_col+5);
        segCol = M(sec_row:sec_row+5, sec_col+2);
        val = sum(segRow)+sum(segCol);
        if val == 2 && sum(segRow)~=1,
            val = 5;
        end
        ultra(sec_row:sec_row+3, sec_col:sec_col+3) = val;
    end
end

M = abs(M-1);
M = M(2:end-1, 2:end-1);

p = ones(dim2,dim1)*(1/n); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bluetooth Setup%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
instrhwinfo('bluetooth');
x = Bluetooth('HC05',1);
fopen(x);
pause(1);
fprintf(x,'r');  % send matlab ready signal
fprintf(x,command);
go = true;
end_command = 666;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%Run Localization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while go 
pause(1); % pause 1 seconds to wait for incoming signal.

while x.bytesAvailable > 0
      sample = fscanf(x,'%s'); % store incoming data (which is converted to string) into variable
      ind1 = find(sample == '.');
      ind2 = find(sample == ':');
      ind3 = find(sample == ';');
      ind4 = find(sample == '!');
      m_u = [m_u, str2double(sample(ind1+1:ind2-1))];
      m_m = [m_m, sample(ind2+1:ind3-1)];
      end_command = str2double(sample(ind3+1:ind4-1));
end

if (LZ == 1)
   fprintf(x,'e');
   disp('L/Z arrived!');
end

if (end_command == 999)
    go = false;
    disp('Program terminated.');
end
 
for k = 1:length(m_m),
    p = sense_u(ultra, M, p, m_u(k));
    [p, heading] = move(p, M, heading, m_m(k));
end

% imagesc(p);
% title(['step: ' num2str(k)]);

p1 = p(5:8,1:4);  %side L/Z entry
p2 = p(1:4,5:8);  %top L/Z entry
sum(p1(:))
sum(p2(:))

if ((sum(p1(:)) > LZ_p) || (sum(p2(:)) > LZ_p))
   fprintf(x,'e');
   LZ = 1;
   disp('L/Z arrived!');
end

p = ones(dim2,dim1)*(1/n); 

end

fclose(x);