class UNICORNCmdInterface
{
public:
    UNICORNCmdInterface();
    ~UNICORNCmdInterface();
    void displayUserInterface();
    int getUserInput();
    cmd_struct_ processUserInput(int choice);
    void sendUNICORNCommand(cmd_struct_ cmd);

protected:
private:
    ros::Publisher unicorn_cmd_sub_;
}