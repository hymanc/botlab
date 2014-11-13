include ../src/common.mk

LCM_JAR = /usr/local/share/java/lcm.jar

ROB550_JAR = $(JAVA_PATH)/rob550.jar
BUILD_PATH = build
SOURCE_PATH = src

# Construct the pathnames to the C types
LCMTYPES_NAMES := $(basename $(shell cd ../lcmtypes && ls *.lcm))
LCMTYPES_JAVA := $(addsuffix .java, $(addprefix $(SOURCE_PATH)/rob550/lcmtypes/, $(LCMTYPES_NAMES)))
LCMTYPES_CLASS := $(addsuffix .class, $(addprefix $(BUILD_PATH)/rob550/lcmtypes/, $(LCMTYPES_NAMES)))

PLUGINS_NAMES := $(basename $(shell cd $(SOURCE_PATH) && ls rob550/plugins/*.java))
PLUGINS_JAVA := $(addsuffix .java, $(addprefix $(SOURCE_PATH)/, $(PLUGINS_NAMES)))
PLUGINS_CLASS := $(addsuffix .class, $(addprefix $(BUILD_PATH)/, $(PLUGINS_NAMES)))

all: $(ROB550_JAR)

$(LCMTYPES_CLASS): $(LCMTYPES_JAVA)
	@echo "[java/lcmtypes]"
	@echo "\t$(LCMTYPES_JAVA)"
	@javac -classpath $(LCM_JAR) -d $(BUILD_PATH) $(LCMTYPES_JAVA)

$(PLUGINS_CLASS): $(PLUGINS_JAVA)
	@echo "[java/plugins]"
	@echo "\t$(PLUGINS_CLASS)"
	@javac -classpath $(BUILD_PATH):$(LCM_JAR) -d $(BUILD_PATH) $(PLUGINS_JAVA)

$(ROB550_JAR): $(LCMTYPES_CLASS) $(PLUGINS_CLASS)
	@echo "[java/rob550.jar]"
	@echo "$(ROB550_JAR)"
	@jar cf $(ROB550_JAR) -C $(BUILD_PATH) .

src/rob550/lcmtypes/%.java: ../lcmtypes/%.lcm
	@echo "\t$<"
	@lcm-gen -j --jpath=src --jdefaultpkg=rob550.lcmtypes $<

builddir:
	@mkdir -p $(BUILD_PATH)

clean:
	@echo "clean [java/lcmtypes]"; rm -f *~ $(LCMTYPES_JAVA)
	@echo "clean [java/build]"; rm -rf build/*
	@echo "clean [java/rob550.jar]"; rm -f $(ROB550_JAR)
