function cd = get_cd(alpha)
    data = load('C_d');
    cd = fixpt_interp1([-8.5:.25:13.75, 14.5,14.75, 15],data.C_d,alpha,sfix(8),2^-3,sfix(16), 2^-14,'Nearest');
end