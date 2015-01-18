library (ggplot2)
library (reshape2)

chTimes = read.csv("ch_times.csv")
plot = ggplot(melt (chTimes, id.var = "problemSize"), aes(x = problemSize, y = value, colour = variable)) + geom_line() + xlab("Problem Size") + ylab("Time")
ggsave ("ch_times.png", plot)
