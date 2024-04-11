pipeline {
  agent {
  // Equivalent to "docker build -f ./.devcontainer/Dockerfile --build-arg name=value ./sr
  dockerfile { // dockerfile parameter
    filename             './.devcontainer/Dockerfile' // filename argument and value
    dir                  './'
    additionalBuildArgs  '--build-arg name=value'
    args                 '-v /tmp:/tmp'
  }

  }
  // Poll SCM
  triggers {
    //Hourly polling
    pollSCM('H */1 * * *')
  }
  stages {
    stage('Building') {
      steps {
        sh '''
          ./start.sh build
        '''
      }
    }
    stage('Quality control') {
      steps {
        echo 'linting ...'
      }
    }
    stage('Testing') {
      steps {
        sh './test.sh'
      }
    }
  }
}
