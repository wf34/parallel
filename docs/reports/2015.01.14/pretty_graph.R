library (ggplot2)
library (reshape2)
library(scales)

chTimes = read.csv("ch_times_2.csv")
plot = ggplot(melt (chTimes, id.var = "problemSize"), aes(x = problemSize, y = value, colour = variable)) + geom_line() + geom_point ()+ xlab("Problem Size") + ylab("Time") + scale_x_continuous(labels = comma)
ggsave ("ch_times.png", plot)
