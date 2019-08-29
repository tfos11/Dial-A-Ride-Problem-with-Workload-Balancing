library(plyr)
library(tidyverse)
library(lubridate)
library(googleway)

#script that takes an input of latitudes and longitudes and uses a Google API and the googleway package to give 
#travel time estimates for the DARP.

key <- #Google API Key

#Create Data

n <- #Number of DARP Requests
index <- 0:(2*n+1)

#Create travel matrix from google

Lats <- #vector of all location latitudes, in the order of the c(depot, pickup locations, delivery locations, depot)

Longs <-#vector of all location longitudes, in the order of the c(depot, pickup locations, delivery locations, depot)
routes <- data.frame(from_lat=1, from_long=2, to_lat=3, to_long = 4, stringsAsFactors = F)

#Create matrix of all possible routes

for (j in 1:(n*2+2)) {
  for (i in 1:(n*2+2)) {
    
    routes[m,1] <- Lats[j]
    routes[m,2] <- Longs[j]
    routes[m,3] <- Lats[i]
    routes[m,4] <- Longs[i]
    m<-m+1
  }
}

#Function to send route start and end points to google. googleway must accept the input as a list.

routelist <- lapply(1:nrow(routes), function(x){
  
  foo <- google_distance(origin = unlist(routes[x, 1:2]),
                         destination = unlist(routes[x, 3:4]),
                         key = key,
                         mode = "driving",
                         simplify = TRUE)
  
  return(foo)
})

#Extract route time from routelist

temp <- sapply(1:nrow(routes), function(x){
  foo <- distance_elements(routelist[[x]])
  return(foo)
})

#Convert list output into a dataframe

temp1 <- data.frame(matrix(unlist(temp), nrow = nrow(routes), byrow = T), stringsAsFactors = F)

#Get travel times and convert to minutes

Travel <- as.numeric(temp1$X4)
remove(temp, temp1)
Travel <- round(Travel/60,0)

#Format travel times to format used by the DARP models
Tij <- as.data.frame(Travel)
Tij$j <- rep(0:((n*2)+1), n*2+2)
Tij$i <- rep(0:((n*2)+1), each=n*2+2)
Tij <- Tij[,c(3,2,1)]
