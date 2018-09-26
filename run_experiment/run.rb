if ENV['ROS_MASTER_URI'] != 'http://nervebaxter.nrv:11311'
  puts "ROS_MASTER_URI is not http://nervebaxter.nrv:11311"
  puts 'Please run `b`'
  exit
end



require 'io/console'
require_relative 'program.rb'





total_trials_for_each = 10

# order = [0, 1, 2]  # p3X p09       p15  p21  p25  p31X  p35  p38
# order = [0, 2, 1]  # p1X p07  p13X p19  p24  p26        p29  p36
# order = [1, 0, 2]  # p4X p10       p16  p22  p27        p32  p39
order = [1, 2, 0]  #     p6        p12  p18  p28        p33  p41
# order = [2, 0, 1]  #     p2        p08  p14  p20        p30  p37
# order = [2, 1, 0]  #     p5        p11  p17  p23        p34  p40




puts 'This handover experiment is about to start. Please check:'
puts ' - Is anaconda uncommented in .bashrc?'
puts ' - leafpad /home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/run_experiment/commands.txt'
puts ' - Is Baxter on and enabled?'
puts ' - Is the force sensor attached firmly?'
puts ' - Did you run `b` before running this?'
puts ' - Did you run `bl`?'
puts ' - Is Baxter\'s hand ready? /home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/cmake-build-debug/devel/lib/handover_moveit/get_almost_ready_node'
puts ' - Is Baxter\'s face normal?'
puts ' - Is Baxter\'s looking at participants ready? /home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/cmake-build-debug/devel/lib/handover_moveit/look_at_participants_node'
puts ' - Did you change the order from previous participant & write down on the questionnaire?'
puts ' - Did you prepare the Amazon gift card?'
puts ' - Did you write participant id on the questionnaire'
puts ' - Is the camera on & has SD cards (STBY) and the 4-camera view ready (select the right profile)?'
puts ' - double check: is the force sensor attached firmly?'
puts ' - record on each camcorder and hit the record button on tv!'
puts

look_at_participants_program = Program.new('experiment_ready_node', "/home/zhao/Dropbox/research17fall/experiment/baxter_ws/src/handover_moveit/cmake-build-debug/devel/lib/handover_moveit/look_at_participants_node")

policies = PolicyProgram.all.values_at *order

puts "Policies (order: #{order.join(',')}): #{policies.join(', ')}"

policies.each_with_index do |policy, nth_policy|
  total_trials_for_each.times do |i|
    n = i + 1
    print "\nPress Enter key to run #{policy.name.upcase} release policy (#{n}/#{total_trials_for_each})"
    gets
    puts Time.now
    policy.run
    puts "#{policy.name.upcase} release policy was just executed"
  end
  look_at_participants_program.run

  puts "#{nth_policy+1}/#{policies.size} is finished."
  puts Time.now
  puts ' - Please ask the participant to fill out the questionnaire for the run'
  puts ' - Please stop recording'
  puts ' - Please double-check if the force sensor attached firmly?'

end


puts
puts ' - Please hand out the end-of-study questionnaire'
puts ' - Please pause all camcorders and the multiplex button'
puts ' - Please copy the output and save it in a file'
puts ' - Please move the video from the camcorder & back up to the Windows machine!'
puts ' - Lastly please give the participant the gift card'
