include ../Rules.mk                                                                 
                                                                                    
APP := transcoder                                                                   
                                          
SRCS := \                                                                           
        video_encode_csvparser.cpp \      
        video_decode_csvparser.cpp \                                                
        transcoder_main.cpp \                                                       
        $(wildcard $(CLASS_DIR)/*.cpp)    
                                                                                    
OBJS := $(SRCS:.cpp=.o)                                                             
                                                                                    
all: $(APP)                                                                         
                                                                                    
$(CLASS_DIR)/%.o: $(CLASS_DIR)/%.cpp                                                
        $(AT)$(MAKE) -C $(CLASS_DIR)                                                
                                                                                    
%.o: %.cpp                                                                          
        @echo "Compiling: $<"                                                       
        $(CPP) $(CPPFLAGS) -c $<                                                    
                                                                                    
$(APP): $(OBJS)                                                                     
        @echo "Linking: $@"                                                         
        $(CPP) -o $@ $(OBJS) $(CPPFLAGS) $(LDFLAGS)
                                                                                    
clean:                                                                              
        $(AT)rm -rf $(APP) $(OBJS)                               