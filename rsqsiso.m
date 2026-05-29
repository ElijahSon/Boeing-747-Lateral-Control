function H = rsqsiso(Ae, Be, Ce, De)

    for rho = 1:10
        K = 1/rho ;
        figure, rlocus(Ae,Be,Ce,De), sgrid; 
    end 
end 
