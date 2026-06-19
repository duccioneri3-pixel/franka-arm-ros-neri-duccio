// planner_bt.cpp — scheletro Behavior Tree (sotto-step 1)
// Scopo: dimostrare che behaviortree_cpp v4 linka, l'albero si costruisce
// da XML e si ticka. Due azioni finte, nessuna logica MoveIt ancora.
// Eseguibile separato: planner_node resta intatto.

#include <iostream>
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

// Azione foglia di prova: stampa e ritorna SUCCESS.
class SayHello : public SyncActionNode
{
public:
  SayHello(const std::string & name, const NodeConfig & config)
  : SyncActionNode(name, config) {}

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("message") };
  }

  NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    std::cout << "[BT] SayHello: " << (msg ? msg.value() : "(no message)") << std::endl;
    return NodeStatus::SUCCESS;
  }
};

// Albero descritto in XML: una Sequence con due azioni finte.
static const char* xml_tree = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <SayHello message="Behavior Tree avviato"/>
      <SayHello message="Secondo nodo eseguito"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

int main()
{
  BehaviorTreeFactory factory;
  factory.registerNodeType<SayHello>("SayHello");

  auto tree = factory.createTreeFromText(xml_tree);

  std::cout << "[BT] Tick dell'albero..." << std::endl;
  NodeStatus result = tree.tickWhileRunning();

  std::cout << "[BT] Risultato finale: " << toStr(result) << std::endl;
  return 0;
}
