library(ggplot2)
bsp4 = read.csv("bsp.csv")

##ggplot(melt(households, id="Year"),       
##              aes(x=Year, y=value, color=variable)) +
##  geom <- line(size=2) + geom <- point(size=5) +  
##    ylab("Percentage of Households")

ggplot(data=bsp4, aes(x = problemSize, y = Time, group=processors)) + geom_line(aes(colour=processors)) + geom_point() + xlab("Problem Size") + ylab("Parallel Time") + ggtitle("LCS problem computation times")
