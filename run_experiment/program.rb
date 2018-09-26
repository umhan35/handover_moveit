
class Program
  attr_accessor :name

  def initialize(name, path)
    @name = name
    @program_path = path
  end

  def check_existence
    if File.exist?(@program_path)
      puts "#{@name} program exists"
    else
      puts "#{@name} program does not exist (#{@program_path})"
    end
  end

  def run
    puts "running #{@program_path}..."
    system(@program_path)
  end
end


class PolicyProgram < Program

  def initialize(policy_name)
    path = "/home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/cmake-build-debug/devel/lib/handover_moveit/#{policy_name}_release_node"
    super(policy_name, path)
  end

  def to_s
    @name
  end

  @@all =
    [
      PolicyProgram.new('rigid'),
      PolicyProgram.new('passive'),
      PolicyProgram.new('proactive')
    ]

  def self.all
    @@all
  end
end