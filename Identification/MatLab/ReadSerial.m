
ARDUINO = serial('/dev/ttyACM0');
ARDUINO.BaudRate = 115200;

FListOpen=fopen('all'); %get the names of the open files

if(~ismember('ARDUINO',FListOpen))
    fopen(ARDUINO);
end

time=[];
input=[];
output=[];
count = 0;



while(count~=3)
    s = fscanf(ARDUINO);
    [values,count] = sscanf(s,"%f,%f,%f\n");
end
time = [time;values(1)];
input = [input;values(2)];
output = [output;values(3)];
fprintf(1,"%s\n",s);

while(time(end)<60000000)
    s = fscanf(ARDUINO);
    [values,count] = sscanf(s,"%f,%f,%f\n");
    if(count~=3)
        continue
    end
    time = [time;values(1)];
    input = [input;values(2)];
    output = [output;values(3)];
    fprintf(1,"%s\n",s);
end

fclose(ARDUINO);
clear ARDUINO count FListOpen s values;

time = time*1e-6;       % pass to seconds



    