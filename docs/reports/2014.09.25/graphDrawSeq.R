library(ggplot2)
bsp4 = read.csv("seq.csv")

##ggplot(melt(households, id="Year"),       
##              aes(x=Year, y=value, color=variable)) +
##  geom <- line(size=2) + geom <- point(size=5) +  
##    ylab("Percentage of Households")

ggplot(data=bsp4, aes(x = problemSize, y = Time)) + geom_line() + geom_point() + xlab("Problem Size") + ylab("Time") + ggtitle("LCS problem sequential computation times")
