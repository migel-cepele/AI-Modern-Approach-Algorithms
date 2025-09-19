#Genetic algorithm search
import bisect
import numpy as np
from State import State

# this algorithm simulates a genetic evolution of an initial population of states.
# based on some parameters, we generate more states based on those initial states, 
# until we find a state whose generated value exceeds the f_thres parameter
# the value of the generated state is computed arbitrarily and some default parameters
# like ngen, pmut and gene_pool

class GA:
    def __init__(self, population):
        self.population = population

    def genetic_search(self, ngen=10, pmut=0.1, n=10):

        states = self.population
        np.random.shuffle(states)

        return self.genetic_algorithm(states)



    def genetic_algorithm(self, population : list[State], gene_pool=['A','T'], f_thres = 500, ngen = 10, pmut = 0.1):
        for i in range(ngen):
            for j in range(len(population)):
                population.append(self.mutate(self.recombine(*self.select(2, population)), gene_pool, pmut))
            
            fittest_individual = self.fitness_threshhold(f_thres, population)
            if fittest_individual:
                return fittest_individual
            
        index = np.argmax([state.value for state in population])
        return population[index].name, population[index].value

    
    def fitness_threshhold(self, f_thres, population : list[State]):
        if not f_thres:
            return None
        # return the index
        fittest_individual = np.argmax([state.value for state in population])
        print("Fittest individual value in population ", population[fittest_individual].value)

        if population[fittest_individual].value >= f_thres:
            return population[fittest_individual].name, population[fittest_individual].value
        
        return None


    def init_population(self, pop_no, gene_pool, state_length):
        # pop_no is number of individuals in population
        # gene pool is list of possible values for individuals
        # state length is the length of each individual

        g = len(gene_pool)
        population = []
        for i in range(pop_no):
            new_individual = [gene_pool[np.random.randint(0,g)] for j in range(state_length)]
            population.append(new_individual)

        return population

    def select(self, r, population : list[State]):
        fitnesses = [item.value for item in population]
        sampler = self.weighted_sampler(population, fitnesses)
        return [sampler() for i in range(r)]
    

    def weighted_sampler(self,seq, weights):
        
        totals = []
        for w in weights:
            totals.append(w + totals[-1] if totals else w)
        return lambda: seq[bisect.bisect(totals, np.random.uniform(0, totals[-1]))]


    def recombine(self, xx: State, yy: State):
        x = xx.name
        y = yy.name
        n = len(x)
        c = np.random.randint(0, n)
        new_val = np.random.randint((xx.value + yy.value)/2, (xx.value + yy.value) * 2/3)
        new_state = State(x[:c] + y[c:], new_val)
        print("New val for state ", new_state.name, " is ", new_val)
        return State(new_state.name, new_val)


    def recombine_uniform(self, x, y):
        n = len(x)
        result = [0] * n # list with size n with all 0
        indexes = np.random.sample(range(n), n) # returns ndarray with size n

        for i in range(n):
            ix = indexes[i]
            result[ix] = x[ix] if i < n/2 else y[ix]

        return ''.join(str(r) for r in result)

    def mutate(self, xx : State, gene_pool, pmut):
        x = xx.name
        if np.random.uniform(0,1) >= pmut:
            return xx
        
        n = len(x)
        g = len(gene_pool)
        c = np.random.randint(0,n)
        r = np.random.randint(0,g)

        new_gene = gene_pool[r]
        new_state = State(x[:c] + new_gene + x[c+1:], xx.value)
        print("Mutation of the state ", new_state.name, " with value ", new_state.value)
        return new_state
        
        

init_population = [
    State("ATCGBTACG", 20), 
    State("GTCGATCGT", 50), 
    State("TGCATGCAT", 30), 
    State("CGTAGCTAG", 100), 
    State("TAGCTAGCA", 85), 
    State("AGCTAGCTA", 90),
    State("CTAGCTAGC", 55),
    State("GATCGATCG", 60),
    State("TCGATCGAT", 20),
    State("CGATCGATC", 50)
    ]

ga = GA(init_population)
print("State with the value higher than the threshhold in GAS is ", ga.genetic_search())